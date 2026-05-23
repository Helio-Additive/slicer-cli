//! Frustum culling and plane intersection testing
//!
//! C++ Reference:
//! - Frustum.hpp (61 lines)
//! - Frustum.cpp (169 lines)
//!
//! This module provides frustum culling functionality for 3D rendering and visibility testing.
//! A frustum is defined by 6 planes and can test intersection with bounding boxes, points,
//! line segments, and triangles.

use crate::geometry::{BoundingBox3F, Point3F};

/// 4D vector type (used for plane coefficients [a, b, c, d])
type Vec4f = [f64; 4];

/// Clip mask bits for each frustum plane
/// Frustum.hpp:47-54
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum FrustumClipMask {
    /// Frustum.hpp:48
    PositiveX = 1 << 0,
    /// Frustum.hpp:49
    NegativeX = 1 << 1,
    /// Frustum.hpp:50
    PositiveY = 1 << 2,
    /// Frustum.hpp:51
    NegativeY = 1 << 3,
    /// Frustum.hpp:52
    PositiveZ = 1 << 4,
    /// Frustum.hpp:53
    NegativeZ = 1 << 5,
}

/// Array of frustum clip masks for all 6 planes
/// Frustum.hpp:56-58
pub const FRUSTUM_CLIP_MASK_ARRAY: [u32; 6] = [
    FrustumClipMask::PositiveX as u32,
    FrustumClipMask::NegativeX as u32,
    FrustumClipMask::PositiveY as u32,
    FrustumClipMask::NegativeY as u32,
    FrustumClipMask::PositiveZ as u32,
    FrustumClipMask::NegativeZ as u32,
];

/// Plane coefficients for each frustum clip plane
/// Frustum.hpp:60
pub const FRUSTUM_CLIP_PLANE: [Vec4f; 6] = [
    [-1.0, 0.0, 0.0, 1.0],
    [1.0, 0.0, 0.0, 1.0],
    [0.0, -1.0, 0.0, 1.0],
    [0.0, 1.0, 0.0, 1.0],
    [0.0, 0.0, -1.0, 1.0],
    [0.0, 0.0, 1.0, 1.0],
];

/// Result of plane intersection test
/// Frustum.hpp:14
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlaneIntersects {
    /// Geometry crosses the plane
    /// Frustum.hpp:14
    Cross = 0,
    /// Geometry is tangent to the plane
    /// Frustum.hpp:14
    Tangent = 1,
    /// Geometry is in front of the plane
    /// Frustum.hpp:14
    Front = 2,
    /// Geometry is behind the plane
    /// Frustum.hpp:14
    Back = 3,
}

/// A plane in 3D space defined by equation ax + by + cz + d = 0
/// Frustum.hpp:13-30
#[derive(Debug, Clone)]
pub struct Plane {
    /// Plane coefficients [a, b, c, d] where ax + by + cz + d = 0
    /// Frustum.hpp:29
    m_abcd: Vec4f,
}

impl Plane {
    /// Create a new plane with default coefficients (0, 0, 0, 0)
    /// Frustum.hpp:13
    pub fn new() -> Self {
        Self {
            m_abcd: [0.0, 0.0, 0.0, 0.0],
        }
    }

    /// Set plane coefficients
    /// Frustum.cpp:5-11
    pub fn set_abcd(&mut self, a: f64, b: f64, c: f64, d: f64) {
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
    pub fn get_abcd(&self) -> Vec4f {
        // Frustum.cpp:15
        self.m_abcd
    }

    /// Normalize the plane equation so that the normal vector (a, b, c) has unit length
    /// Frustum.cpp:18-25
    pub fn normalize(&mut self) {
        // Calculate magnitude of normal vector
        // Frustum.cpp:20
        let mag = (self.m_abcd[0] * self.m_abcd[0]
            + self.m_abcd[1] * self.m_abcd[1]
            + self.m_abcd[2] * self.m_abcd[2])
            .sqrt();

        // Normalize all coefficients by magnitude
        // Frustum.cpp:21-24
        self.m_abcd[0] /= mag;
        self.m_abcd[1] /= mag;
        self.m_abcd[2] /= mag;
        self.m_abcd[3] /= mag;
    }

    /// Calculate signed distance from point to plane
    /// Frustum.cpp:27-36
    pub fn distance(&self, pt: &Point3F) -> f64 {
        // Initialize result to zero
        // Frustum.cpp:29
        let mut result = 0.0f64;

        // Accumulate dot product of point with normal (a, b, c)
        // Frustum.cpp:30-32
        result += pt.x * self.m_abcd[0];
        result += pt.y * self.m_abcd[1];
        result += pt.z * self.m_abcd[2];

        // Add the d coefficient
        // Frustum.cpp:34
        result += self.m_abcd[3];

        // Frustum.cpp:36
        result
    }

    /// Test intersection of plane with axis-aligned bounding box
    /// See: https://cgvr.cs.uni-bremen.de/teaching/cg_literatur/lighthouse3d_view_frustum_culling/index.html
    /// Frustum.cpp:38-76
    pub fn intersects_box(&self, bbox: &BoundingBox3F) -> PlaneIntersects {
        // Find the "positive vertex" - the corner of the box most in the direction of the plane normal
        // Frustum.cpp:42-47
        let mut positive_v = bbox.min;
        if self.m_abcd[0] > 0.0 {
            positive_v.x = bbox.max.x;
        }
        if self.m_abcd[1] > 0.0 {
            positive_v.y = bbox.max.y;
        }
        if self.m_abcd[2] > 0.0 {
            positive_v.z = bbox.max.z;
        }

        // If positive vertex is behind plane, entire box is behind
        // Frustum.cpp:49-52
        let dis_positive = self.distance(&positive_v);
        if dis_positive < 0.0 {
            return PlaneIntersects::Back;
        }

        // Find the "negative vertex" - the corner most opposite to the plane normal
        // Frustum.cpp:54-59
        let mut negative_v = bbox.max;
        if self.m_abcd[0] > 0.0 {
            negative_v.x = bbox.min.x;
        }
        if self.m_abcd[1] > 0.0 {
            negative_v.y = bbox.min.y;
        }
        if self.m_abcd[2] > 0.0 {
            negative_v.z = bbox.min.z;
        }

        // Check if negative vertex is behind plane
        // Frustum.cpp:61
        let dis_negative = self.distance(&negative_v);

        // If negative vertex is also behind, box crosses the plane
        // Frustum.cpp:63-66
        if dis_negative < 0.0 {
            return PlaneIntersects::Cross;
        }

        // Otherwise, entire box is in front of plane
        // Frustum.cpp:68
        PlaneIntersects::Front
    }

    /// Test intersection of plane with a point
    /// Frustum.cpp:77-83
    pub fn intersects_point(&self, p0: &Point3F) -> PlaneIntersects {
        // Calculate distance from point to plane
        // Frustum.cpp:79
        let d = self.distance(p0);

        // Frustum.cpp:80-82
        if d == 0.0 {
            PlaneIntersects::Tangent
        } else if d > 0.0 {
            PlaneIntersects::Front
        } else {
            PlaneIntersects::Back
        }
    }

    /// Test intersection of plane with a line segment
    /// Frustum.cpp:84-96
    pub fn intersects_segment(&self, p0: &Point3F, p1: &Point3F) -> PlaneIntersects {
        // Test both endpoints
        // Frustum.cpp:86-87
        let state0 = self.intersects_point(p0);
        let state1 = self.intersects_point(p1);

        // If both endpoints have same state, segment has that state
        // Frustum.cpp:88-90
        if state0 == state1 {
            return state0;
        }

        // If either endpoint is tangent, segment is tangent
        // Frustum.cpp:91-93
        if state0 == PlaneIntersects::Tangent || state1 == PlaneIntersects::Tangent {
            return PlaneIntersects::Tangent;
        }

        // Otherwise, endpoints are on opposite sides, so segment crosses
        // Frustum.cpp:95
        PlaneIntersects::Cross
    }

    /// Test intersection of plane with a triangle
    /// Frustum.cpp:97-111
    pub fn intersects_triangle(&self, p0: &Point3F, p1: &Point3F, p2: &Point3F) -> PlaneIntersects {
        // Test all three edges of the triangle
        // Frustum.cpp:99-101
        let state0 = self.intersects_segment(p0, p1);
        let state1 = self.intersects_segment(p0, p2);
        let state2 = self.intersects_segment(p1, p2);

        // If all edges have same state, triangle has that state
        // Frustum.cpp:103-104
        if state0 == state1 && state0 == state2 {
            return state0;
        }

        // If any edge crosses, triangle crosses
        // Frustum.cpp:106-108
        if state0 == PlaneIntersects::Cross
            || state1 == PlaneIntersects::Cross
            || state2 == PlaneIntersects::Cross
        {
            return PlaneIntersects::Cross;
        }

        // Otherwise, triangle is tangent
        // Frustum.cpp:110
        PlaneIntersects::Tangent
    }
}

impl Default for Plane {
    fn default() -> Self {
        Self::new()
    }
}

/// A view frustum defined by 6 planes (left, right, top, bottom, near, far)
/// Frustum.hpp:8-44
#[derive(Debug, Clone)]
pub struct Frustum {
    /// The 6 planes defining the frustum
    /// Frustum.hpp:43
    pub planes: [Plane; 6],
}

impl Frustum {
    /// Create a new frustum with default planes
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

    /// Test if frustum intersects with an axis-aligned bounding box
    /// Returns false if box is completely outside frustum (culled)
    /// Frustum.cpp:113-123
    pub fn intersects_box(&self, bbox: &BoundingBox3F) -> bool {
        // Test box against each plane
        // Frustum.cpp:115-120
        for plane in &self.planes {
            let rt = plane.intersects_box(bbox);
            // If box is behind any plane, it's outside the frustum
            // Frustum.cpp:117-119
            if rt == PlaneIntersects::Back {
                return false;
            }
        }

        // Box intersects or is inside frustum
        // Frustum.cpp:122
        true
    }

    /// Test if frustum intersects with a point
    /// Frustum.cpp:125-131
    pub fn intersects_point(&self, p0: &Point3F) -> bool {
        // Test point against each plane
        // Frustum.cpp:126-128
        for plane in &self.planes {
            if plane.intersects_point(p0) == PlaneIntersects::Back {
                return false;
            }
        }
        // Frustum.cpp:129
        true
    }

    /// Test if frustum intersects with a line segment
    /// Frustum.cpp:133-143
    pub fn intersects_segment(&self, p0: &Point3F, p1: &Point3F) -> bool {
        // Test segment against each plane
        // Frustum.cpp:135-140
        for plane in &self.planes {
            if plane.intersects_segment(p0, p1) == PlaneIntersects::Back {
                return false;
            }
        }
        // Frustum.cpp:141
        true
    }

    /// Test if frustum intersects with a triangle
    /// Frustum.cpp:145-155
    pub fn intersects_triangle(&self, p0: &Point3F, p1: &Point3F, p2: &Point3F) -> bool {
        // Test triangle against each plane
        // Frustum.cpp:147-152
        for plane in &self.planes {
            if plane.intersects_triangle(p0, p1, p2) == PlaneIntersects::Back {
                return false;
            }
        }
        // Frustum.cpp:153
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
    fn test_plane_normalize() {
        let mut plane = Plane::new();
        plane.set_abcd(3.0, 4.0, 0.0, 5.0);
        plane.normalize();
        let abcd = plane.get_abcd();
        let mag = (abcd[0] * abcd[0] + abcd[1] * abcd[1] + abcd[2] * abcd[2]).sqrt();
        assert!((mag - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_plane_distance() {
        let mut plane = Plane::new();
        // Plane: z = 0 (xy-plane)
        plane.set_abcd(0.0, 0.0, 1.0, 0.0);

        let p1 = Point3F::new(0.0, 0.0, 5.0);
        let d1 = plane.distance(&p1);
        assert_eq!(d1, 5.0);

        let p2 = Point3F::new(0.0, 0.0, -3.0);
        let d2 = plane.distance(&p2);
        assert_eq!(d2, -3.0);
    }

    #[test]
    fn test_plane_intersects_point() {
        let mut plane = Plane::new();
        plane.set_abcd(0.0, 0.0, 1.0, -1.0); // z = 1

        let p_front = Point3F::new(0.0, 0.0, 2.0);
        assert_eq!(plane.intersects_point(&p_front), PlaneIntersects::Front);

        let p_back = Point3F::new(0.0, 0.0, 0.0);
        assert_eq!(plane.intersects_point(&p_back), PlaneIntersects::Back);

        let p_on = Point3F::new(0.0, 0.0, 1.0);
        assert_eq!(plane.intersects_point(&p_on), PlaneIntersects::Tangent);
    }

    #[test]
    fn test_plane_intersects_segment() {
        let mut plane = Plane::new();
        plane.set_abcd(0.0, 0.0, 1.0, 0.0); // z = 0

        let p1 = Point3F::new(0.0, 0.0, 1.0);
        let p2 = Point3F::new(0.0, 0.0, -1.0);
        assert_eq!(plane.intersects_segment(&p1, &p2), PlaneIntersects::Cross);

        let p3 = Point3F::new(0.0, 0.0, 1.0);
        let p4 = Point3F::new(0.0, 0.0, 2.0);
        assert_eq!(plane.intersects_segment(&p3, &p4), PlaneIntersects::Front);
    }

    #[test]
    fn test_frustum_new() {
        let frustum = Frustum::new();
        assert_eq!(frustum.planes.len(), 6);
    }

    #[test]
    fn test_frustum_intersects_point() {
        let mut frustum = Frustum::new();
        // Set up a simple frustum: all planes pass through origin, normals point outward
        // This creates a frustum that contains points near the origin
        for i in 0..6 {
            frustum.planes[i].set_abcd(
                FRUSTUM_CLIP_PLANE[i][0],
                FRUSTUM_CLIP_PLANE[i][1],
                FRUSTUM_CLIP_PLANE[i][2],
                FRUSTUM_CLIP_PLANE[i][3],
            );
        }

        let p_inside = Point3F::new(0.0, 0.0, 0.0);
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
