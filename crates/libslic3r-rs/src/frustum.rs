//! Frustum culling and plane intersection testing
//!
//! C++ Reference:
//! - Frustum.hpp (61 lines)
//! - Frustum.cpp (151 lines)
//!
//! This module provides frustum culling functionality for 3D rendering and visibility testing.
//! A frustum is defined by 6 planes and can test intersection with bounding boxes, points,
//! line segments, and triangles.
//!
//! Fidelity note: the C++ `Frustum::Plane` stores its coefficients as a `Vec4f`
//! (`Eigen::Matrix<float,4,1>`) and performs all plane math in single precision
//! (`float`). The point/segment/triangle overloads take `Vec3f` (float). Only the
//! bounding-box overload reads `BoundingBoxf3` (double) corners and then
//! `.cast<float>()`s them before computing the distance. We mirror that exactly:
//! `Vec4f`/`Vec3f` are `f32`-based and the box corners (`f64`) are cast to `f32`.

use crate::geometry::BoundingBox3F;

/// 3D vector type (float), mirrors C++ `Vec3f = Eigen::Matrix<float,3,1>`.
/// Point.hpp:44
type Vec3f = [f32; 3];

/// 4D vector type (float), mirrors C++ `Vec4f = Eigen::Matrix<float,4,1>`.
/// Used for plane coefficients [a, b, c, d].
/// Point.hpp:48
type Vec4f = [f32; 4];

/// Clip mask bits for each frustum plane
/// Frustum.hpp:45-52
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum FrustumClipMask {
    /// Frustum.hpp:46
    PositiveX = 1 << 0,
    /// Frustum.hpp:47
    NegativeX = 1 << 1,
    /// Frustum.hpp:48
    PositiveY = 1 << 2,
    /// Frustum.hpp:49
    NegativeY = 1 << 3,
    /// Frustum.hpp:50
    PositiveZ = 1 << 4,
    /// Frustum.hpp:51
    NegativeZ = 1 << 5,
}

/// Array of frustum clip masks for all 6 planes
/// Frustum.hpp:54-56
pub const FRUSTUM_CLIP_MASK_ARRAY: [i32; 6] = [
    FrustumClipMask::PositiveX as i32,
    FrustumClipMask::NegativeX as i32,
    FrustumClipMask::PositiveY as i32,
    FrustumClipMask::NegativeY as i32,
    FrustumClipMask::PositiveZ as i32,
    FrustumClipMask::NegativeZ as i32,
];

/// Plane coefficients for each frustum clip plane
/// Frustum.hpp:58
pub const FRUSTUM_CLIP_PLANE: [Vec4f; 6] = [
    [-1.0, 0.0, 0.0, 1.0],
    [1.0, 0.0, 0.0, 1.0],
    [0.0, -1.0, 0.0, 1.0],
    [0.0, 1.0, 0.0, 1.0],
    [0.0, 0.0, -1.0, 1.0],
    [0.0, 0.0, 1.0, 1.0],
];

/// Result of plane intersection test
/// Frustum.hpp:15
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlaneIntersects {
    /// Frustum.hpp:15
    IntersectsCross = 0,
    /// Frustum.hpp:15
    IntersectsTangent = 1,
    /// Frustum.hpp:15
    IntersectsFront = 2,
    /// Frustum.hpp:15
    IntersectsBack = 3,
}

/// A plane in 3D space defined by equation ax + by + cz + d = 0
/// Frustum.hpp:13-32
#[derive(Debug, Clone)]
pub struct Plane {
    /// Plane coefficients [a, b, c, d] where ax + by + cz + d = 0
    /// Frustum.hpp:31
    m_abcd: Vec4f,
}

impl Plane {
    /// Create a new plane with default (zero) coefficients.
    /// Frustum.hpp:13
    pub fn new() -> Self {
        Self {
            m_abcd: [0.0, 0.0, 0.0, 0.0],
        }
    }

    /// Set plane coefficients
    /// Frustum.cpp:5-11
    pub fn set_abcd(&mut self, a: f32, b: f32, c: f32, d: f32) {
        // Frustum.cpp:7
        self.m_abcd[0] = a;
        // Frustum.cpp:8
        self.m_abcd[1] = b;
        // Frustum.cpp:9
        self.m_abcd[2] = c;
        // Frustum.cpp:10
        self.m_abcd[3] = d;
    }

    /// Get plane coefficients
    /// Frustum.cpp:13-16
    pub fn get_abcd(&self) -> &Vec4f {
        // Frustum.cpp:15
        &self.m_abcd
    }

    /// Normalize the plane equation so that the normal vector (a, b, c) has unit length.
    /// Frustum.cpp:18-26
    pub fn normailze(&mut self) {
        // Frustum.cpp:20
        let mag: f32;
        // Frustum.cpp:21
        mag = (self.m_abcd[0] * self.m_abcd[0]
            + self.m_abcd[1] * self.m_abcd[1]
            + self.m_abcd[2] * self.m_abcd[2])
            .sqrt();
        // Frustum.cpp:22
        self.m_abcd[0] = self.m_abcd[0] / mag;
        // Frustum.cpp:23
        self.m_abcd[1] = self.m_abcd[1] / mag;
        // Frustum.cpp:24
        self.m_abcd[2] = self.m_abcd[2] / mag;
        // Frustum.cpp:25
        self.m_abcd[3] = self.m_abcd[3] / mag;
    }

    /// Calculate signed distance from point to plane.
    /// Frustum.cpp:28-38
    pub fn distance(&self, pt: &Vec3f) -> f32 {
        // Frustum.cpp:30
        let mut result: f32 = 0.0;
        // Frustum.cpp:31-33
        for i in 0..3 {
            result += pt[i] * self.m_abcd[i];
        }

        // Frustum.cpp:35
        result += self.m_abcd[3];

        // Frustum.cpp:37
        result
    }

    /// Test intersection of plane with axis-aligned bounding box.
    /// see https://cgvr.cs.uni-bremen.de/teaching/cg_literatur/lighthouse3d_view_frustum_culling/index.html
    /// Frustum.cpp:40-74
    pub fn intersects_box(&self, box_: &BoundingBox3F) -> PlaneIntersects {
        // Frustum.cpp:44
        let mut positive_v = box_.min;
        // Frustum.cpp:45-46
        if self.m_abcd[0] > 0.0 {
            positive_v.x = box_.max.x();
        }
        // Frustum.cpp:47-48
        if self.m_abcd[1] > 0.0 {
            positive_v.y = box_.max.y();
        }
        // Frustum.cpp:49-50
        if self.m_abcd[2] > 0.0 {
            positive_v.z = box_.max.z();
        }

        // Frustum.cpp:52
        let dis_positive = self.distance(&[
            positive_v.x as f32,
            positive_v.y as f32,
            positive_v.z as f32,
        ]);
        // Frustum.cpp:53-56
        if dis_positive < 0.0 {
            return PlaneIntersects::IntersectsBack;
        }

        // Frustum.cpp:58
        let mut negitive_v = box_.max;
        // Frustum.cpp:59-60
        if self.m_abcd[0] > 0.0 {
            negitive_v.x = box_.min.x();
        }
        // Frustum.cpp:61-62
        if self.m_abcd[1] > 0.0 {
            negitive_v.y = box_.min.y();
        }
        // Frustum.cpp:63-64
        if self.m_abcd[2] > 0.0 {
            negitive_v.z = box_.min.z();
        }

        // Frustum.cpp:66
        let dis_negitive = self.distance(&[
            negitive_v.x as f32,
            negitive_v.y as f32,
            negitive_v.z as f32,
        ]);

        // Frustum.cpp:68-71
        if dis_negitive < 0.0 {
            return PlaneIntersects::IntersectsCross;
        }

        // Frustum.cpp:73
        PlaneIntersects::IntersectsFront
    }

    /// Test intersection of plane with a point (world space).
    /// Frustum.cpp:75-82
    pub fn intersects_point(&self, p0: &Vec3f) -> PlaneIntersects {
        // Frustum.cpp:77
        let d = self.distance(p0);
        // Frustum.cpp:78-80
        if d == 0.0 {
            return PlaneIntersects::IntersectsTangent;
        }
        // Frustum.cpp:81
        if d > 0.0 {
            PlaneIntersects::IntersectsFront
        } else {
            PlaneIntersects::IntersectsBack
        }
    }

    /// Test intersection of plane with a line segment (world space).
    /// Frustum.cpp:83-95
    pub fn intersects_segment(&self, p0: &Vec3f, p1: &Vec3f) -> PlaneIntersects {
        // Frustum.cpp:85
        let state0 = self.intersects_point(p0);
        // Frustum.cpp:86
        let state1 = self.intersects_point(p1);
        // Frustum.cpp:87-89
        if state0 == state1 {
            return state0;
        }
        // Frustum.cpp:90-92
        if state0 == PlaneIntersects::IntersectsTangent
            || state1 == PlaneIntersects::IntersectsTangent
        {
            return PlaneIntersects::IntersectsTangent;
        }

        // Frustum.cpp:94
        PlaneIntersects::IntersectsCross
    }

    /// Test intersection of plane with a triangle (world space).
    /// Frustum.cpp:96-110
    pub fn intersects_triangle(&self, p0: &Vec3f, p1: &Vec3f, p2: &Vec3f) -> PlaneIntersects {
        // Frustum.cpp:98
        let state0 = self.intersects_segment(p0, p1);
        // Frustum.cpp:99
        let state1 = self.intersects_segment(p0, p2);
        // Frustum.cpp:100
        let state2 = self.intersects_segment(p1, p2);

        // Frustum.cpp:102-103
        if state0 == state1 && state0 == state2 {
            return state0;
        }

        // Frustum.cpp:105-107
        if state0 == PlaneIntersects::IntersectsCross
            || state1 == PlaneIntersects::IntersectsCross
            || state2 == PlaneIntersects::IntersectsCross
        {
            return PlaneIntersects::IntersectsCross;
        }

        // Frustum.cpp:109
        PlaneIntersects::IntersectsTangent
    }
}

impl Default for Plane {
    fn default() -> Self {
        Self::new()
    }
}

/// A view frustum defined by 6 planes.
/// Frustum.hpp:7-43
#[derive(Debug, Clone)]
pub struct Frustum {
    /// The 6 planes defining the frustum.
    /// Frustum.hpp:42
    pub planes: [Plane; 6],
}

impl Frustum {
    /// Create a new frustum with default planes.
    /// Frustum.hpp:10
    pub fn new() -> Self {
        Self {
            planes: [
                Plane::new(),
                Plane::new(),
                Plane::new(),
                Plane::new(),
                Plane::new(),
                Plane::new(),
            ],
        }
    }

    /// Test if frustum intersects with an axis-aligned bounding box.
    /// Frustum.cpp:112-122
    pub fn intersects_box(&self, box_: &BoundingBox3F) -> bool {
        // Frustum.cpp:114-119
        for plane in &self.planes {
            // Frustum.cpp:115
            let rt = plane.intersects_box(box_);
            // Frustum.cpp:116-118
            if PlaneIntersects::IntersectsBack == rt {
                return false;
            }
        }

        // Frustum.cpp:121
        true
    }

    /// Test if frustum intersects with a point (world space).
    /// Frustum.cpp:124-129
    pub fn intersects_point(&self, p0: &Vec3f) -> bool {
        // Frustum.cpp:125-127
        for plane in &self.planes {
            if plane.intersects_point(p0) == PlaneIntersects::IntersectsBack {
                return false;
            }
        }
        // Frustum.cpp:128
        true
    }

    /// Test if frustum intersects with a line segment (world space).
    /// Frustum.cpp:131-139
    pub fn intersects_segment(&self, p0: &Vec3f, p1: &Vec3f) -> bool {
        // Frustum.cpp:133-138
        for plane in &self.planes {
            if plane.intersects_segment(p0, p1) == PlaneIntersects::IntersectsBack {
                return false;
            }
        }
        // Frustum.cpp:138
        true
    }

    /// Test if frustum intersects with a triangle (world space).
    /// Frustum.cpp:141-149
    pub fn intersects_triangle(&self, p0: &Vec3f, p1: &Vec3f, p2: &Vec3f) -> bool {
        // Frustum.cpp:143-148
        for plane in &self.planes {
            if plane.intersects_triangle(p0, p1, p2) == PlaneIntersects::IntersectsBack {
                return false;
            }
        }
        // Frustum.cpp:148
        true
    }
}

impl Default for Frustum {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_plane_new() {
        let plane = Plane::new();
        assert_eq!(plane.get_abcd()[0], 0.0);
        assert_eq!(plane.get_abcd()[1], 0.0);
        assert_eq!(plane.get_abcd()[2], 0.0);
        assert_eq!(plane.get_abcd()[3], 0.0);
    }

    #[test]
    fn test_plane_set_abcd() {
        let mut plane = Plane::new();
        plane.set_abcd(1.0, 2.0, 3.0, 4.0);
        assert_eq!(plane.get_abcd()[0], 1.0);
        assert_eq!(plane.get_abcd()[1], 2.0);
        assert_eq!(plane.get_abcd()[2], 3.0);
        assert_eq!(plane.get_abcd()[3], 4.0);
    }

    #[test]
    fn test_plane_normailze() {
        let mut plane = Plane::new();
        plane.set_abcd(3.0, 4.0, 0.0, 5.0);
        plane.normailze();
        let abcd = plane.get_abcd();
        let mag = (abcd[0] * abcd[0] + abcd[1] * abcd[1] + abcd[2] * abcd[2]).sqrt();
        assert!((mag - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_plane_distance() {
        let mut plane = Plane::new();
        // Plane: z = 0 (xy-plane)
        plane.set_abcd(0.0, 0.0, 1.0, 0.0);

        let p1: Vec3f = [0.0, 0.0, 5.0];
        let d1 = plane.distance(&p1);
        assert_eq!(d1, 5.0);

        let p2: Vec3f = [0.0, 0.0, -3.0];
        let d2 = plane.distance(&p2);
        assert_eq!(d2, -3.0);
    }

    #[test]
    fn test_plane_intersects_point() {
        let mut plane = Plane::new();
        plane.set_abcd(0.0, 0.0, 1.0, -1.0); // z = 1

        let p_front: Vec3f = [0.0, 0.0, 2.0];
        assert_eq!(
            plane.intersects_point(&p_front),
            PlaneIntersects::IntersectsFront
        );

        let p_back: Vec3f = [0.0, 0.0, 0.0];
        assert_eq!(
            plane.intersects_point(&p_back),
            PlaneIntersects::IntersectsBack
        );

        let p_on: Vec3f = [0.0, 0.0, 1.0];
        assert_eq!(
            plane.intersects_point(&p_on),
            PlaneIntersects::IntersectsTangent
        );
    }

    #[test]
    fn test_plane_intersects_segment() {
        let mut plane = Plane::new();
        plane.set_abcd(0.0, 0.0, 1.0, 0.0); // z = 0

        let p1: Vec3f = [0.0, 0.0, 1.0];
        let p2: Vec3f = [0.0, 0.0, -1.0];
        assert_eq!(
            plane.intersects_segment(&p1, &p2),
            PlaneIntersects::IntersectsCross
        );

        let p3: Vec3f = [0.0, 0.0, 1.0];
        let p4: Vec3f = [0.0, 0.0, 2.0];
        assert_eq!(
            plane.intersects_segment(&p3, &p4),
            PlaneIntersects::IntersectsFront
        );
    }

    #[test]
    fn test_frustum_new() {
        let frustum = Frustum::new();
        assert_eq!(frustum.planes.len(), 6);
    }

    #[test]
    fn test_frustum_intersects_point() {
        let mut frustum = Frustum::new();
        // Set up a simple frustum from the clip planes.
        for i in 0..6 {
            frustum.planes[i].set_abcd(
                FRUSTUM_CLIP_PLANE[i][0],
                FRUSTUM_CLIP_PLANE[i][1],
                FRUSTUM_CLIP_PLANE[i][2],
                FRUSTUM_CLIP_PLANE[i][3],
            );
        }

        let p_inside: Vec3f = [0.0, 0.0, 0.0];
        assert!(frustum.intersects_point(&p_inside));
    }

    #[test]
    fn test_clip_mask_values() {
        assert_eq!(FrustumClipMask::PositiveX as u32, 1);
        assert_eq!(FrustumClipMask::NegativeX as u32, 2);
        assert_eq!(FrustumClipMask::PositiveY as u32, 4);
        assert_eq!(FrustumClipMask::NegativeY as u32, 8);
        assert_eq!(FrustumClipMask::PositiveZ as u32, 16);
        assert_eq!(FrustumClipMask::NegativeZ as u32, 32);
    }
}
