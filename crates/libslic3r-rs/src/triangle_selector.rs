//! Faithful 1:1 port of `TriangleSelector.{cpp,hpp}` from BambuStudio libslic3r.
//!
//! C++ Reference:
//! - src/libslic3r/TriangleSelector.hpp (554 lines at the pin)
//! - src/libslic3r/TriangleSelector.cpp (2634 lines at the pin)
//!
//! Much of this file was first ported from an earlier TriangleSelector.cpp; some line citations
//! still carry that file's numbers. Where the pin changed behaviour, the pin's code is behind a
//! faithful gate with the earlier code as the off arm: `TS_PAINT_STREAM_GUARD` (deserialize /
//! static has_facets), `EXTRUDER_MAX_PIN` (`ExtruderMax` = 96), `TS_FLOAT_ANGLE_LIMITS` (float
//! angle limits of the brush and fill tools), `TS_EIGEN_FP` (the float evaluation order of the C++
//! build). The NDEBUG-only consistency asserts run only with the diagnostic switch `TS_VERIFY`
//! (see `verify_triangle_neighbors`). Compared with the pin in
//! tests/triangle_selector_native_parity.rs.
//!
//! Following class holds information about selected triangles. It also has power
//! to recursively subdivide the triangles and make the selection finer.
//!
//! Fidelity notes (byte-exact G-code parity):
//! - `coord_t` -> `i64`, `coordf_t` -> `f64`. Mesh vertices are `stl_vertex`
//!   (Eigen `Vec3f`, i.e. `f32`); triangle indices are `stl_triangle_vertex_indices`
//!   (Eigen `Vec3i`). We operate on `crate::triangle_mesh::indexed_triangle_set`
//!   (= the C++ `TriangleMesh::its`) directly, since the C++ class only ever reads
//!   `m_mesh.its`.
//! - C++ `Transform3d` is an Eigen `Transform<double,3,Affine>`. We model it as
//!   `nalgebra::Matrix4<f64>` and `Transform3f` as `nalgebra::Matrix4<f32>`. The
//!   point-transform `trafo * v` follows Eigen affine semantics: the 3x3 linear block
//!   times `v` plus the translation column. `trafo.linear()` is the top-left 3x3 block.
//! - `EnforcerBlockerType` is `int8_t` in C++ (Model.hpp). We model it as a thin
//!   `i8` newtype to preserve all integer/serialization arithmetic exactly.
//! - The cursor class hierarchy (`Cursor` -> `SinglePointCursor`/`DoublePointCursor`
//!   -> `Sphere`/`Circle`/`HeightRange`/`Capsule3D`/`Capsule2D`) is modeled with a
//!   `CursorKind` enum dispatched by hand, preserving virtual-call behaviour exactly.

// #define PRUSASLICER_TRIANGLE_SELECTOR_DEBUG  // TriangleSelector.hpp:4

use crate::geometry::{deg2rad, Transformation};
use crate::libslic3r::EPSILON;
use crate::triangle_mesh::{
    its_face_neighbors, its_face_normals, indexed_triangle_set, TriangleMesh, Vec2i, Vec3f, Vec3i,
};
use crate::utils::{next_highest_power_of_2, next_idx_modulo, prev_idx_modulo};
use nalgebra::{Matrix3, Matrix4};
use std::collections::{BTreeSet, VecDeque};

/// C++ `Transform3d` = Eigen `Transform<double,3,Affine>`.
/// Point.hpp
pub type Transform3d = Matrix4<f64>;
/// C++ `Transform3f` = Eigen `Transform<float,3,Affine>`.
/// Point.hpp
pub type Transform3f = Matrix4<f32>;
/// C++ `Matrix3f` = Eigen `Matrix<float,3,3>`.
/// Point.hpp
pub type Matrix3f = Matrix3<f32>;

/// `stl_vertex` (admesh/stl.h => `Vec3f`).
pub type StlVertex = Vec3f;
/// `stl_triangle_vertex_indices` (Eigen `Vec3i`).
pub type StlTriangleVertexIndices = Vec3i;

// ----------------------------------------------------------------------------
// EnforcerBlockerType (Model.hpp:713-749) — needed by this file, not yet ported.
// ----------------------------------------------------------------------------

/// `enum class EnforcerBlockerType : int8_t` (Model.hpp:713)
///
/// Modeled as an `i8` newtype so all the serialization / shift arithmetic in this
/// file (`(int)state`, `state - 1`, `EnforcerBlockerType(n)`, comparisons) is
/// reproduced byte-exactly. The named constants below mirror the C++ enumerators.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct EnforcerBlockerType(pub i8);

impl EnforcerBlockerType {
    // Maximum is 3. The value is serialized in TriangleSelector into 2 bits.
    pub const NONE: EnforcerBlockerType = EnforcerBlockerType(0);
    pub const ENFORCER: EnforcerBlockerType = EnforcerBlockerType(1);
    pub const BLOCKER: EnforcerBlockerType = EnforcerBlockerType(2);
    pub const FUZZY_SKIN: EnforcerBlockerType = EnforcerBlockerType(1);
    // Maximum is 15. The value is serialized in TriangleSelector into 6 bits using a 2 bit prefix code.
    pub const EXTRUDER1: EnforcerBlockerType = EnforcerBlockerType(1);
    pub const EXTRUDER2: EnforcerBlockerType = EnforcerBlockerType(2);
    /// The `ExtruderMax` of an earlier Bambu Studio (32). Kept as the value callers pass for
    /// "the default `max_ebt`" (`deserialize`'s default argument is `ExtruderMax`); with the
    /// gate `EXTRUDER_MAX_PIN` on, `deserialize` reads it as [`EnforcerBlockerType::extruder_max`].
    pub const EXTRUDER_MAX: EnforcerBlockerType = EnforcerBlockerType(32);
    /// Model.hpp:717-720,765 at the pin: `ExtruderMax = int8_t(MAX_FILAMENT_COUNT)`,
    /// `MAX_FILAMENT_COUNT = MAX_PHYSICAL_FILAMENT_COUNT (32) + MAX_MIXED_FILAMENT_COUNT (64)`.
    pub const EXTRUDER_MAX_AT_PIN: EnforcerBlockerType = EnforcerBlockerType(96);

    /// `EnforcerBlockerType::ExtruderMax`. The pin's value (96) behind the faithful gate
    /// `EXTRUDER_MAX_PIN`; the earlier crate value (32) with `EXTRUDER_MAX_PIN=0`.
    pub fn extruder_max() -> EnforcerBlockerType {
        if crate::faithful_gate("EXTRUDER_MAX_PIN") {
            Self::EXTRUDER_MAX_AT_PIN
        } else {
            Self::EXTRUDER_MAX
        }
    }
}

// ----------------------------------------------------------------------------
// TriangleSelector.cpp:17-48 — anonymous namespace: paint bit-stream guards
// ----------------------------------------------------------------------------

/// TriangleSelector.cpp:18-33 `read_paint_nibble`: paint bit-stream is 4 bits per nibble.
/// Malicious / truncated 3MF must not OOB.
pub fn read_paint_nibble(bits: &[bool], ibit: &mut i32, nibble: &mut i32) -> bool {
    // TriangleSelector.cpp:21
    if *ibit < 0 || *ibit > bits.len() as i32 - 4 {
        // TriangleSelector.cpp:22-26 — static std::once_flag + std::call_once
        static ONCE: std::sync::Once = std::sync::Once::new();
        let (ibit_v, size) = (*ibit, bits.len());
        ONCE.call_once(|| {
            crate::boost_log_trivial!(Warning, "TriangleSelector paint stream truncated or out of range: ibit=", ibit_v, " bits=", size);
        });
        return false; // TriangleSelector.cpp:27
    }
    *nibble = 0; // TriangleSelector.cpp:29
    for i in 0..4 {
        // TriangleSelector.cpp:30-31
        *nibble |= (bits[*ibit as usize] as i32) << i;
        *ibit += 1;
    }
    true // TriangleSelector.cpp:32
}

/// TriangleSelector.cpp:35-47 `clamp_paint_state`
pub fn clamp_paint_state(state_i: i32, max_ebt: EnforcerBlockerType) -> EnforcerBlockerType {
    let max_i = max_ebt.0 as i32; // TriangleSelector.cpp:37
    if state_i < 0 || state_i > max_i {
        // TriangleSelector.cpp:39-43 — static std::once_flag + std::call_once
        static ONCE: std::sync::Once = std::sync::Once::new();
        ONCE.call_once(|| {
            crate::boost_log_trivial!(Warning, "TriangleSelector paint state out of range: state=", state_i, " max=", max_i);
        });
        return EnforcerBlockerType::NONE; // TriangleSelector.cpp:44
    }
    EnforcerBlockerType(state_i as i8) // TriangleSelector.cpp:46
}

// ----------------------------------------------------------------------------
// Small helpers (Eigen affine semantics).
// ----------------------------------------------------------------------------

/// `Slic3r::sqr(x)` — square.
#[inline]
fn sqr_f32(x: f32) -> f32 {
    x * x
}

/// Eigen affine point transform: `trafo * v` (rotation/scale + translation).
///
/// Gate `TS_EIGEN_FP` (faithful): per row `fma(m2, v2, fma(m1, v1, m0 * v0)) + m3`, the order the
/// C++ build evaluates a `Transform3f` times a `Vec3f` in (as `triangle_mesh_slicer::transform3f_mul_vec3f`
/// measures it). Off: the earlier nalgebra product, unfused.
#[inline]
fn transform_point_f32(trafo: &Transform3f, v: &Vec3f) -> Vec3f {
    if eigen_fp() {
        let row = |r: usize| trafo[(r, 2)].mul_add(v.z, trafo[(r, 1)].mul_add(v.y, trafo[(r, 0)] * v.x)) + trafo[(r, 3)];
        return Vec3f::new(row(0), row(1), row(2));
    }
    let linear = trafo.fixed_view::<3, 3>(0, 0);
    let translation = trafo.fixed_view::<3, 1>(0, 3);
    (linear * v) + translation
}

/// `Vec3f::dot`. Gate `TS_EIGEN_FP` (faithful): `fma(a.z, b.z, a.x * b.x + a.y * b.y)` as the C++
/// build evaluates it (R880); off: the earlier nalgebra dot, `(a.x * b.x + a.y * b.y) + a.z * b.z`.
#[inline]
fn dotf(a: &Vec3f, b: &Vec3f) -> f32 {
    if eigen_fp() {
        eigen_dot_f32(a, b)
    } else {
        a.dot(b)
    }
}

/// `Vec3f::cross`. Gate `TS_EIGEN_FP` (faithful): each component `fma(a, b, -(c * d))` (R876);
/// off: the earlier nalgebra cross, unfused.
#[inline]
fn crossf(a: &Vec3f, b: &Vec3f) -> Vec3f {
    if eigen_fp() {
        Vec3f::new(a.y.mul_add(b.z, -(a.z * b.y)), a.z.mul_add(b.x, -(a.x * b.z)), a.x.mul_add(b.y, -(a.y * b.x)))
    } else {
        a.cross(b)
    }
}

/// `a * b - c * d` in one C++ expression. Gate `TS_EIGEN_FP` (faithful): `fma(a, b, -(c * d))`;
/// off: unfused.
#[inline]
fn fms(a: f32, b: f32, c: f32, d: f32) -> f32 {
    if eigen_fp() {
        a.mul_add(b, -(c * d))
    } else {
        a * b - c * d
    }
}

/// `m + t * n` in one C++ expression. Gate `TS_EIGEN_FP` (faithful): `fma(t, n, m)`; off: unfused.
#[inline]
fn fma_add(m: f32, t: f32, n: f32) -> f32 {
    if eigen_fp() {
        t.mul_add(n, m)
    } else {
        m + t * n
    }
}

/// Eigen affine point transform with f64 trafo applied to an f32 point: `trafo * v`.
#[inline]
fn transform_point_f64_to_f32(trafo: &Transform3d, v: &Vec3f) -> Vec3f {
    let vd = nalgebra::Vector3::<f64>::new(v.x as f64, v.y as f64, v.z as f64);
    let linear = trafo.fixed_view::<3, 3>(0, 0);
    let translation = trafo.fixed_view::<3, 1>(0, 3);
    let r = (linear * vd) + translation;
    Vec3f::new(r.x as f32, r.y as f32, r.z as f32)
}

/// `trafo.linear()` — top-left 3x3 block.
#[inline]
fn linear_f32(trafo: &Transform3f) -> Matrix3f {
    trafo.fixed_view::<3, 3>(0, 0).into_owned()
}

/// Geometry.hpp:295 `template<typename T> constexpr T deg2rad(const T angle)` for `T = float`:
/// `float(PI) * angle / float(180.0)`, in float. TriangleSelector.cpp calls it with float angles
/// and takes the float `cos` of the result.
#[inline]
fn deg2rad_f32(angle: f32) -> f32 {
    (std::f64::consts::PI as f32) * angle / 180.0f32
}

/// Gate `TS_FLOAT_ANGLE_LIMITS` (faithful): the angle limits of select_patch(),
/// seed_fill_select_triangles() and bucket_fill_select_triangles() computed as the pin computes
/// them, `deg2rad<float>` and `cos(float)`. Off: the earlier double computation.
#[inline]
fn float_angle_limits() -> bool {
    crate::faithful_gate("TS_FLOAT_ANGLE_LIMITS")
}

// ============================================================================
// TriangleSelector.cpp:13-50 — test_line_inside_sphere
// ============================================================================

// Check if the line is whole inside the sphere, or it is partially inside (intersecting) the sphere.
// Inspired by Christer Ericson's Real-Time Collision Detection, pp. 177-179.
// TriangleSelector.cpp:15
fn test_line_inside_sphere(line_a: &Vec3f, line_b: &Vec3f, sphere_p: &Vec3f, sphere_radius: f32) -> bool {
    let sphere_radius_sqr = sqr_f32(sphere_radius); // TriangleSelector.cpp:17
    let line_dir = line_b - line_a; // n  // TriangleSelector.cpp:18
    let origins_diff = line_a - sphere_p; // m  // TriangleSelector.cpp:19

    let m_dot_m = dotf(&origins_diff, &origins_diff); // TriangleSelector.cpp:21
    // Check if any of the end-points of the line is inside the sphere.
    // TriangleSelector.cpp:23
    if m_dot_m <= sphere_radius_sqr || (line_b - sphere_p).norm_squared() <= sphere_radius_sqr {
        return true;
    }

    // Check if the infinite line is going through the sphere.
    let n_dot_n = dotf(&line_dir, &line_dir); // TriangleSelector.cpp:27
    let m_dot_n = dotf(&origins_diff, &line_dir); // TriangleSelector.cpp:28

    let eq_a = n_dot_n; // TriangleSelector.cpp:30
    let eq_b = m_dot_n; // TriangleSelector.cpp:31
    let eq_c = m_dot_m - sphere_radius_sqr; // TriangleSelector.cpp:32

    let discr = fms(eq_b, eq_b, eq_a, eq_c); // TriangleSelector.cpp:71
    // A negative discriminant corresponds to the infinite line infinite not going through the sphere.
    if discr < 0.0 {
        // TriangleSelector.cpp:36
        return false;
    }

    // Check if the finite line is going through the sphere.
    let discr_sqrt = discr.sqrt(); // TriangleSelector.cpp:40
    let t1 = (-eq_b - discr_sqrt) / eq_a; // TriangleSelector.cpp:41
    if 0.0 <= t1 && t1 <= 1.0 {
        // TriangleSelector.cpp:42
        return true;
    }

    let t2 = (-eq_b + discr_sqrt) / eq_a; // TriangleSelector.cpp:45
    if 0.0 <= t2 && t2 <= 1.0 && discr_sqrt > 0.0 {
        // TriangleSelector.cpp:46
        return true;
    }

    false // TriangleSelector.cpp:49
}

// ============================================================================
// TriangleSelector.cpp:52-105 — test_line_inside_cylinder
// ============================================================================

// Check if the line is whole inside the finite cylinder, or it is partially inside (intersecting) the finite cylinder.
// Inspired by Christer Ericson's Real-Time Collision Detection, pp. 194-198.
// TriangleSelector.cpp:54
fn test_line_inside_cylinder(line_a: &Vec3f, line_b: &Vec3f, cylinder_p: &Vec3f, cylinder_q: &Vec3f, cylinder_radius: f32) -> bool {
    debug_assert!(cylinder_p != cylinder_q); // TriangleSelector.cpp:56
    let cylinder_dir = cylinder_q - cylinder_p; // d  // TriangleSelector.cpp:57
    // TriangleSelector.cpp:58-65
    let is_point_inside_finite_cylinder = |pt: &Vec3f| -> bool {
        let first_center_diff = cylinder_p - pt; // TriangleSelector.cpp:59
        let second_center_diff = cylinder_q - pt; // TriangleSelector.cpp:60
        // First, check if the point pt is laying between planes defined by cylinder_p and cylinder_q.
        // Then check if it is inside the cylinder between cylinder_p and cylinder_q.
        // TriangleSelector.cpp:63-64
        dotf(&first_center_diff, &cylinder_dir) <= 0.0
            && dotf(&second_center_diff, &cylinder_dir) >= 0.0
            && (crossf(&first_center_diff, &cylinder_dir).norm() / cylinder_dir.norm()) <= cylinder_radius
    };

    // Check if any of the end-points of the line is inside the cylinder.
    // TriangleSelector.cpp:68
    if is_point_inside_finite_cylinder(line_a) || is_point_inside_finite_cylinder(line_b) {
        return true;
    }

    // Check if the line is going through the cylinder.
    let origins_diff = line_a - cylinder_p; // m  // TriangleSelector.cpp:72
    let line_dir = line_b - line_a; // n  // TriangleSelector.cpp:73

    let m_dot_d = dotf(&origins_diff, &cylinder_dir); // TriangleSelector.cpp:75
    let n_dot_d = dotf(&line_dir, &cylinder_dir); // TriangleSelector.cpp:76
    let d_dot_d = dotf(&cylinder_dir, &cylinder_dir); // TriangleSelector.cpp:77

    let n_dot_n = dotf(&line_dir, &line_dir); // TriangleSelector.cpp:79
    let m_dot_n = dotf(&origins_diff, &line_dir); // TriangleSelector.cpp:80
    let m_dot_m = dotf(&origins_diff, &origins_diff); // TriangleSelector.cpp:81

    let eq_a = fms(d_dot_d, n_dot_n, n_dot_d, n_dot_d); // TriangleSelector.cpp:120
    let eq_b = fms(d_dot_d, m_dot_n, n_dot_d, m_dot_d); // TriangleSelector.cpp:121
    let eq_c = fms(d_dot_d, m_dot_m - sqr_f32(cylinder_radius), m_dot_d, m_dot_d); // TriangleSelector.cpp:122

    let discr = fms(eq_b, eq_b, eq_a, eq_c); // TriangleSelector.cpp:124
    // A negative discriminant corresponds to the infinite line not going through the infinite cylinder.
    if discr < 0.0 {
        // TriangleSelector.cpp:89
        return false;
    }

    // Check if the finite line is going through the finite cylinder.
    let discr_sqrt = discr.sqrt(); // TriangleSelector.cpp:93
    let t1 = (-eq_b - discr_sqrt) / eq_a; // TriangleSelector.cpp:94
    if 0.0 <= t1 && t1 <= 1.0 {
        // TriangleSelector.cpp:95-96
        let cylinder_endcap_t1 = fma_add(m_dot_d, t1, n_dot_d);
        if 0.0 <= cylinder_endcap_t1 && cylinder_endcap_t1 <= d_dot_d {
            return true; // TriangleSelector.cpp:97
        }
    }

    let t2 = (-eq_b + discr_sqrt) / eq_a; // TriangleSelector.cpp:99
    if 0.0 <= t2 && t2 <= 1.0 {
        // TriangleSelector.cpp:100-101
        let cylinder_endcap_t2 = fma_add(m_dot_d, t2, n_dot_d);
        if 0.0 <= cylinder_endcap_t2 && cylinder_endcap_t2 <= d_dot_d {
            return true; // TriangleSelector.cpp:102
        }
    }

    false // TriangleSelector.cpp:104
}

// ============================================================================
// TriangleSelector.cpp:107-117 — test_line_inside_capsule
// ============================================================================

// Check if the line is whole inside the capsule, or it is partially inside (intersecting) the capsule.
// TriangleSelector.cpp:108
fn test_line_inside_capsule(line_a: &Vec3f, line_b: &Vec3f, capsule_p: &Vec3f, capsule_q: &Vec3f, capsule_radius: f32) -> bool {
    debug_assert!(capsule_p != capsule_q); // TriangleSelector.cpp:109

    // Check if the line intersect any of the spheres forming the capsule.
    // TriangleSelector.cpp:112
    if test_line_inside_sphere(line_a, line_b, capsule_p, capsule_radius) || test_line_inside_sphere(line_a, line_b, capsule_q, capsule_radius) {
        return true;
    }

    // Check if the line intersects the cylinder between the centers of the spheres.
    // TriangleSelector.cpp:116
    test_line_inside_cylinder(line_a, line_b, capsule_p, capsule_q, capsule_radius)
}

// ============================================================================
// TriangleSelector.hpp:404-407 — Partition
// ============================================================================

/// `enum class Partition { First, Second };` (TriangleSelector.hpp:404-407)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Partition {
    First,
    Second,
}

// ============================================================================
// TriangleSelector.hpp:23-30 — CursorType
// ============================================================================

/// `enum CursorType` (TriangleSelector.hpp:23-30)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CursorType {
    Circle,
    Sphere,
    Pointer,
    // BBS
    HeightRange,
    GapFill,
}

// ============================================================================
// TriangleSelector.hpp:32-42 — ClippingPlane
// ============================================================================

/// `struct ClippingPlane` (TriangleSelector.hpp:32-42)
#[derive(Debug, Clone, Copy)]
pub struct ClippingPlane {
    pub normal: Vec3f,
    pub offset: f32,
}

impl Default for ClippingPlane {
    fn default() -> Self {
        // TriangleSelector.hpp:36 — ClippingPlane() : normal{0,0,1}, offset{FLT_MAX}
        ClippingPlane {
            normal: Vec3f::new(0.0, 0.0, 1.0),
            offset: f32::MAX,
        }
    }
}

impl ClippingPlane {
    /// TriangleSelector.hpp:36
    pub fn new() -> Self {
        Self::default()
    }

    /// `explicit ClippingPlane(const std::array<float, 4> &clp)` (TriangleSelector.hpp:37)
    pub fn from_array(clp: &[f32; 4]) -> Self {
        ClippingPlane {
            normal: Vec3f::new(clp[0], clp[1], clp[2]),
            offset: clp[3],
        }
    }

    /// TriangleSelector.hpp:39 — bool is_active() const { return offset != FLT_MAX; }
    #[inline]
    pub fn is_active(&self) -> bool {
        self.offset != f32::MAX
    }

    /// TriangleSelector.hpp:41 — bool is_mesh_point_clipped(const Vec3f &point) const
    #[inline]
    pub fn is_mesh_point_clipped(&self, point: &Vec3f) -> bool {
        dotf(&self.normal, point) - self.offset > 0.0
    }
}

// ============================================================================
// TriangleSelector.hpp:296-360 — Triangle and Vertex
// ============================================================================

/// `class Triangle` (TriangleSelector.hpp:296-351). Triangle and info about how it's split.
#[derive(Debug, Clone)]
pub struct Triangle {
    // Indices into m_vertices.  // TriangleSelector.hpp:310
    pub verts_idxs: [i32; 3],
    // Index of the source triangle at the initial (unsplit) mesh.  // TriangleSelector.hpp:313
    pub source_triangle: i32,
    // Children triangles.  // TriangleSelector.hpp:316
    pub children: [i32; 4],

    // Packing the rest of member variables into 4 bytes.  // TriangleSelector.hpp:342
    number_of_splits: i8,
    // Index of a vertex opposite to the split edge (for number_of_splits == 1)
    // or index of a vertex shared by the two split edges (for number_of_splits == 2).
    // For number_of_splits == 3, special_side_idx is always zero.  // TriangleSelector.hpp:346
    special_side_idx: i8,
    state: EnforcerBlockerType, // TriangleSelector.hpp:347
    m_selected_by_seed_fill: bool, // TriangleSelector.hpp:348
    // Is this triangle valid or marked to be removed?  // TriangleSelector.hpp:350
    m_valid: bool,
}

impl Triangle {
    // Use TriangleSelector::push_triangle to create a new triangle.
    // TriangleSelector.hpp:300-308
    pub fn new(a: i32, b: i32, c: i32, source_triangle: i32, init_state: EnforcerBlockerType) -> Self {
        Triangle {
            verts_idxs: [a, b, c],
            source_triangle,
            children: [0, 0, 0, 0],
            number_of_splits: 0,
            special_side_idx: 0,
            state: init_state,
            // Initialize bit fields. Default member initializers are not supported by C++17.
            m_selected_by_seed_fill: false, // TriangleSelector.hpp:306
            m_valid: true, // TriangleSelector.hpp:307
        }
    }

    // Set the division type.
    // sides_to_split==-1 : just restore previous split  // TriangleSelector.cpp:159
    fn set_division(&mut self, sides_to_split: i32, special_side_idx: i32) {
        debug_assert!(sides_to_split >= 0 && sides_to_split <= 3); // TriangleSelector.cpp:162
        debug_assert!(special_side_idx >= 0 && special_side_idx < 3); // TriangleSelector.cpp:163
        debug_assert!(sides_to_split == 1 || sides_to_split == 2 || special_side_idx == 0); // TriangleSelector.cpp:164
        self.number_of_splits = sides_to_split as i8; // TriangleSelector.cpp:165
        self.special_side_idx = special_side_idx as i8; // TriangleSelector.cpp:166
    }

    // Get/set current state.  // TriangleSelector.hpp:322
    fn set_state(&mut self, ty: EnforcerBlockerType) {
        debug_assert!(!self.is_split());
        self.state = ty;
    }

    // TriangleSelector.hpp:323
    #[inline]
    pub fn get_state(&self) -> EnforcerBlockerType {
        self.state
    }

    // Set if the triangle has been selected or unselected by seed fill.  // TriangleSelector.cpp:169
    fn select_by_seed_fill(&mut self) {
        debug_assert!(!self.is_split()); // TriangleSelector.cpp:171
        self.m_selected_by_seed_fill = true; // TriangleSelector.cpp:172
    }

    // TriangleSelector.cpp:175
    fn unselect_by_seed_fill(&mut self) {
        debug_assert!(!self.is_split()); // TriangleSelector.cpp:177
        self.m_selected_by_seed_fill = false; // TriangleSelector.cpp:178
    }

    // Get if the triangle has been selected or not by seed fill.  // TriangleSelector.cpp:181
    fn is_selected_by_seed_fill(&self) -> bool {
        debug_assert!(!self.is_split()); // TriangleSelector.cpp:183
        self.m_selected_by_seed_fill // TriangleSelector.cpp:184
    }

    // Is this triangle valid or marked to be removed?  // TriangleSelector.hpp:332
    #[inline]
    pub fn valid(&self) -> bool {
        self.m_valid
    }

    // Get info on how it's split.  // TriangleSelector.hpp:334
    #[inline]
    pub fn is_split(&self) -> bool {
        self.number_of_split_sides() != 0
    }

    // TriangleSelector.hpp:335
    #[inline]
    pub fn number_of_split_sides(&self) -> i32 {
        self.number_of_splits as i32
    }

    // TriangleSelector.hpp:336
    #[inline]
    fn special_side(&self) -> i32 {
        debug_assert!(self.is_split());
        self.special_side_idx as i32
    }
}

/// `struct Vertex` (TriangleSelector.hpp:353-360)
#[derive(Debug, Clone)]
pub struct Vertex {
    pub v: StlVertex,
    pub ref_cnt: i32,
}

impl Vertex {
    // explicit Vertex(const stl_vertex& vert) : v{vert}, ref_cnt{0}  // TriangleSelector.hpp:354-357
    pub fn new(vert: StlVertex) -> Self {
        Vertex { v: vert, ref_cnt: 0 }
    }
}

// NOTE: The free list trick in undivide_triangle/push_triangle/triangle_midpoint_or_allocate
// stores the int free-list head into the f32 bytes of `m_vertices[iv].v[0]` via memcpy.
// We reproduce that bit-pattern by reinterpreting the f32 lane as the i32 head.
#[inline]
fn read_i32_from_f32(x: f32) -> i32 {
    i32::from_ne_bytes(x.to_ne_bytes())
}

#[inline]
fn write_i32_to_f32(x: i32) -> f32 {
    f32::from_ne_bytes(x.to_ne_bytes())
}

// ============================================================================
// TriangleSelector.hpp:44-208 / .cpp Cursor hierarchy
// ============================================================================

/// Shared base data for `class Cursor` (TriangleSelector.hpp:60-72).
#[derive(Debug, Clone)]
pub struct CursorData {
    pub trafo: Transform3f,
    pub source: Vec3f,
    pub uniform_scaling: bool,
    pub trafo_normal: Transform3f, // C++ stores Transform3f; only .linear() block used.
    pub radius: f32,
    pub radius_sqr: f32,
    pub dir: Vec3f,
    // Clipping plane to limit painting to not clipped facets only.  // TriangleSelector.hpp:72
    pub clipping_plane: ClippingPlane,
}

/// The concrete cursor variant. Mirrors the C++ virtual hierarchy
/// (`Sphere`/`Circle`/`HeightRange`/`Capsule3D`/`Capsule2D`).
#[derive(Debug, Clone)]
pub enum CursorShape {
    // SinglePointCursor adds `center` (TriangleSelector.hpp:102).
    Sphere { center: Vec3f },
    Circle { center: Vec3f },
    // BBS HeightRange (TriangleSelector.hpp:159-177): center is (0,0,0), adds m_z_world / m_height.
    HeightRange { center: Vec3f, m_z_world: f32, m_height: f32 },
    // DoublePointCursor adds first_center/second_center (TriangleSelector.hpp:125-126).
    Capsule3D { first_center: Vec3f, second_center: Vec3f },
    Capsule2D { first_center: Vec3f, second_center: Vec3f },
}

/// `class Cursor` plus its subclasses, as a single owning value.
#[derive(Debug, Clone)]
pub struct Cursor {
    pub base: CursorData,
    pub shape: CursorShape,
}

impl Cursor {
    // explicit Cursor(const Vec3f &source_, float radius_world, const Transform3d &trafo_, const ClippingPlane &clipping_plane_)
    // TriangleSelector.cpp:2038-2056
    fn base_new(source_: &Vec3f, radius_world: f32, trafo_: &Transform3d, clipping_plane_: &ClippingPlane) -> CursorData {
        // TriangleSelector.cpp:2039 — source{source_}, trafo{trafo_.cast<float>()}, clipping_plane{clipping_plane_}
        let trafo: Transform3f = trafo_.cast::<f32>();
        let mut source = *source_;
        let clipping_plane = *clipping_plane_;
        let radius;
        let radius_sqr;
        let uniform_scaling;
        let mut trafo_normal = Transform3f::zeros();

        // TriangleSelector.cpp:2041 — Vec3d sf = Geometry::Transformation(trafo_).get_scaling_factor();
        let sf = Transformation::from_transform(*trafo_).get_scaling_factor();
        // TriangleSelector.cpp:2042 — if (is_approx(sf(0), sf(1)) && is_approx(sf(1), sf(2)))
        if crate::geometry::geometry::is_approx(sf[0], sf[1])
            && crate::geometry::geometry::is_approx(sf[1], sf[2])
        {
            radius = (radius_world as f64 / sf[0]) as f32; // TriangleSelector.cpp:2043
            radius_sqr = ((radius_world as f64 / sf[0]) * (radius_world as f64 / sf[0])) as f32; // TriangleSelector.cpp:2044
            uniform_scaling = true; // TriangleSelector.cpp:2045
        } else {
            // In case that the transformation is non-uniform, all checks whether
            // something is inside the cursor should be done in world coords.
            // First transform source in world coords and remember that we did this.
            source = transform_point_f32(&trafo, &source); // TriangleSelector.cpp:2050
            uniform_scaling = false; // TriangleSelector.cpp:2051
            radius = radius_world; // TriangleSelector.cpp:2052
            radius_sqr = sqr_f32(radius_world); // TriangleSelector.cpp:2053
            // trafo_normal = trafo.linear().inverse().transpose();  // TriangleSelector.cpp:2054
            let lin = linear_f32(&trafo);
            let inv = lin.try_inverse().unwrap_or_else(Matrix3f::zeros);
            let tn = inv.transpose();
            trafo_normal.fixed_view_mut::<3, 3>(0, 0).copy_from(&tn);
        }

        CursorData {
            trafo,
            source,
            uniform_scaling,
            trafo_normal,
            radius,
            radius_sqr,
            dir: Vec3f::new(0.0, 0.0, 0.0), // TriangleSelector.hpp:70
            clipping_plane,
        }
    }

    // SinglePointCursor(const Vec3f& center_, const Vec3f& source_, float radius_world, const Transform3d& trafo_, const ClippingPlane &clipping_plane_)
    // TriangleSelector.cpp:2058-2069
    fn single_point_new(center_: &Vec3f, source_: &Vec3f, radius_world: f32, trafo_: &Transform3d, clipping_plane_: &ClippingPlane) -> (CursorData, Vec3f) {
        let mut base = Self::base_new(source_, radius_world, trafo_, clipping_plane_);
        let mut center = *center_;
        // In case that the transformation is non-uniform, all checks whether
        // something is inside the cursor should be done in world coords.
        // Because of the center is transformed.
        // TriangleSelector.cpp:2064
        if !base.uniform_scaling {
            center = transform_point_f32(&base.trafo, &center); // TriangleSelector.cpp:2065
        }
        // Calculate dir, in whatever coords is appropriate.  // TriangleSelector.cpp:2068
        base.dir = (center - base.source).normalize();
        (base, center)
    }

    // DoublePointCursor(...)  // TriangleSelector.cpp:2071-2081
    fn double_point_new(first_center_: &Vec3f, second_center_: &Vec3f, source_: &Vec3f, radius_world: f32, trafo_: &Transform3d, clipping_plane_: &ClippingPlane) -> (CursorData, Vec3f, Vec3f) {
        let mut base = Self::base_new(source_, radius_world, trafo_, clipping_plane_);
        let mut first_center = *first_center_;
        let mut second_center = *second_center_;
        // TriangleSelector.cpp:2074
        if !base.uniform_scaling {
            first_center = transform_point_f32(&base.trafo, first_center_); // TriangleSelector.cpp:2075
            second_center = transform_point_f32(&base.trafo, second_center_); // TriangleSelector.cpp:2076
        }
        // Calculate dir, in whatever coords is appropriate.  // TriangleSelector.cpp:2080
        base.dir = (first_center - base.source).normalize();
        (base, first_center, second_center)
    }

    // Sphere(center_, source_, radius_world, trafo_, clipping_plane_)  // TriangleSelector.hpp:133
    pub fn new_sphere(center_: &Vec3f, source_: &Vec3f, radius_world: f32, trafo_: &Transform3d, clipping_plane_: &ClippingPlane) -> Cursor {
        let (base, center) = Self::single_point_new(center_, source_, radius_world, trafo_, clipping_plane_);
        Cursor { base, shape: CursorShape::Sphere { center } }
    }

    // Circle(center_, source_, radius_world, trafo_, clipping_plane_)  // TriangleSelector.hpp:146
    pub fn new_circle(center_: &Vec3f, source_: &Vec3f, radius_world: f32, trafo_: &Transform3d, clipping_plane_: &ClippingPlane) -> Cursor {
        let (base, center) = Self::single_point_new(center_, source_, radius_world, trafo_, clipping_plane_);
        Cursor { base, shape: CursorShape::Circle { center } }
    }

    // HeightRange(float z_world_, const Vec3f &source_, float height_, const Transform3d &trafo_, const ClippingPlane &clipping_plane_)
    // TriangleSelector.cpp:1129-1138
    pub fn new_height_range(z_world_: f32, source_: &Vec3f, height_: f32, trafo_: &Transform3d, clipping_plane_: &ClippingPlane) -> Cursor {
        // SinglePointCursor(Vec3f(0,0,0), source_, 1.f, trafo_, clipping_plane_), m_z_world(z_world_), m_height(height_)
        let (mut base, mut center) = Self::single_point_new(&Vec3f::new(0.0, 0.0, 0.0), source_, 1.0, trafo_, clipping_plane_);
        let m_z_world = z_world_;
        let m_height = height_;
        base.uniform_scaling = false; // TriangleSelector.cpp:1132 — HeightRange must use world cs
        // overwrite base
        base.source = transform_point_f32(&base.trafo, &base.source); // TriangleSelector.cpp:1134
        base.radius = height_; // TriangleSelector.cpp:1135
        base.radius_sqr = sqr_f32(height_); // TriangleSelector.cpp:1136
        // trafo_normal = trafo.linear().inverse().transpose();  // TriangleSelector.cpp:1137
        let lin = linear_f32(&base.trafo);
        let inv = lin.try_inverse().unwrap_or_else(Matrix3f::zeros);
        let tn = inv.transpose();
        base.trafo_normal = Transform3f::zeros();
        base.trafo_normal.fixed_view_mut::<3, 3>(0, 0).copy_from(&tn);
        // NOTE: the SinglePointCursor ctor already recomputed dir from (0,0,0)-center & source;
        // HeightRange leaves dir as set by the base ctor (center stays (0,0,0) under uniform_scaling=false
        // path was bypassed: single_point_new computed dir BEFORE this overwrite). center kept for parity.
        let _ = &mut center;
        Cursor { base, shape: CursorShape::HeightRange { center, m_z_world, m_height } }
    }

    // Capsule3D(...)  // TriangleSelector.hpp:183
    pub fn new_capsule3d(first_center_: &Vec3f, second_center_: &Vec3f, source_: &Vec3f, radius_world: f32, trafo_: &Transform3d, clipping_plane_: &ClippingPlane) -> Cursor {
        let (base, first_center, second_center) = Self::double_point_new(first_center_, second_center_, source_, radius_world, trafo_, clipping_plane_);
        Cursor { base, shape: CursorShape::Capsule3D { first_center, second_center } }
    }

    // Capsule2D(...)  // TriangleSelector.hpp:197
    pub fn new_capsule2d(first_center_: &Vec3f, second_center_: &Vec3f, source_: &Vec3f, radius_world: f32, trafo_: &Transform3d, clipping_plane_: &ClippingPlane) -> Cursor {
        let (base, first_center, second_center) = Self::double_point_new(first_center_, second_center_, source_, radius_world, trafo_, clipping_plane_);
        Cursor { base, shape: CursorShape::Capsule2D { first_center, second_center } }
    }

    // SinglePointCursor::cursor_factory (CIRCLE / SPHERE)  // TriangleSelector.hpp:86-93
    pub fn single_point_cursor_factory(center: &Vec3f, camera_pos: &Vec3f, cursor_radius: f32, cursor_type: CursorType, trafo_matrix: &Transform3d, clipping_plane: &ClippingPlane) -> Cursor {
        SinglePointCursor::cursor_factory(center, camera_pos, cursor_radius, cursor_type, trafo_matrix, clipping_plane)
    }

    // SinglePointCursor::cursor_factory (HEIGHT_RANGE)  // TriangleSelector.hpp:95-98
    pub fn height_range_cursor_factory(z_world: f32, camera_pos: &Vec3f, height: f32, trafo_matrix: &Transform3d, clipping_plane: &ClippingPlane) -> Cursor {
        SinglePointCursor::cursor_factory_height_range(z_world, camera_pos, height, trafo_matrix, clipping_plane)
    }

    // DoublePointCursor::cursor_factory (CIRCLE / SPHERE)  // TriangleSelector.hpp:114-121
    pub fn double_point_cursor_factory(first_center: &Vec3f, second_center: &Vec3f, camera_pos: &Vec3f, cursor_radius: f32, cursor_type: CursorType, trafo_matrix: &Transform3d, clipping_plane: &ClippingPlane) -> Cursor {
        DoublePointCursor::cursor_factory(first_center, second_center, camera_pos, cursor_radius, cursor_type, trafo_matrix, clipping_plane)
    }

    // Is pointer in a triangle? (Triangle overload)  // TriangleSelector.cpp:1055-1060
    fn is_pointer_in_triangle_tr(&self, tr: &Triangle, vertices: &[Vertex]) -> bool {
        let p1 = &vertices[tr.verts_idxs[0] as usize].v;
        let p2 = &vertices[tr.verts_idxs[1] as usize].v;
        let p3 = &vertices[tr.verts_idxs[2] as usize].v;
        self.is_pointer_in_triangle(p1, p2, p3)
    }

    // virtual bool is_pointer_in_triangle(p1, p2, p3) const  — dispatch
    fn is_pointer_in_triangle(&self, p1: &Vec3f, p2: &Vec3f, p3: &Vec3f) -> bool {
        match &self.shape {
            // SinglePointCursor::is_pointer_in_triangle  // TriangleSelector.cpp:2185-2188
            CursorShape::Sphere { center } | CursorShape::Circle { center } => {
                is_circle_pointer_inside_triangle(p1, p2, p3, center, &self.base.dir, self.base.uniform_scaling, &self.base.trafo)
            }
            // HeightRange::is_pointer_in_triangle  // TriangleSelector.cpp:1141-1144 — return false
            CursorShape::HeightRange { .. } => false,
            // DoublePointCursor::is_pointer_in_triangle  // TriangleSelector.cpp:2191-2195
            CursorShape::Capsule3D { first_center, second_center } | CursorShape::Capsule2D { first_center, second_center } => {
                is_circle_pointer_inside_triangle(p1, p2, p3, first_center, &self.base.dir, self.base.uniform_scaling, &self.base.trafo)
                    || is_circle_pointer_inside_triangle(p1, p2, p3, second_center, &self.base.dir, self.base.uniform_scaling, &self.base.trafo)
            }
        }
    }

    // virtual int vertices_inside(const Triangle &tr, const std::vector<Vertex> &vertices) const
    // How many vertices of a triangle are inside the circle?  // TriangleSelector.cpp:1073-1081
    fn vertices_inside(&self, tr: &Triangle, vertices: &[Vertex]) -> i32 {
        let mut inside = 0; // TriangleSelector.cpp:1075
        for i in 0..3usize {
            // TriangleSelector.cpp:1077
            if self.is_mesh_point_inside(&vertices[tr.verts_idxs[i] as usize].v) {
                inside += 1;
            }
        }
        inside // TriangleSelector.cpp:1080
    }

    // virtual bool is_mesh_point_inside(const Vec3f &point) const — dispatch
    fn is_mesh_point_inside(&self, point: &Vec3f) -> bool {
        match &self.shape {
            // Sphere::is_mesh_point_inside  // TriangleSelector.cpp:2090-2097
            CursorShape::Sphere { center } => {
                let transformed_point = if self.base.uniform_scaling { *point } else { transform_point_f32(&self.base.trafo, point) };
                if (center - transformed_point).norm_squared() < self.base.radius_sqr {
                    return is_mesh_point_not_clipped(point, &self.base.clipping_plane);
                }
                false
            }
            // Circle::is_mesh_point_inside  // TriangleSelector.cpp:2100-2109
            CursorShape::Circle { center } => {
                let transformed_point = if self.base.uniform_scaling { *point } else { transform_point_f32(&self.base.trafo, point) };
                let diff = center - transformed_point;
                if (diff - dotf(&diff, &self.base.dir) * self.base.dir).norm_squared() < self.base.radius_sqr {
                    return is_mesh_point_not_clipped(point, &self.base.clipping_plane);
                }
                false
            }
            // HeightRange::is_mesh_point_inside  // TriangleSelector.cpp:1146-1155
            CursorShape::HeightRange { m_z_world, m_height, .. } => {
                // just use 40% edge limit as tolerance
                let tolerance: f32 = 0.02; // TriangleSelector.cpp:1149
                let transformed_point = transform_point_f32(&self.base.trafo, point); // TriangleSelector.cpp:1150
                let top_z = m_z_world + m_height + tolerance; // TriangleSelector.cpp:1151
                let bot_z = m_z_world - tolerance; // TriangleSelector.cpp:1152
                transformed_point.z > bot_z && transformed_point.z < top_z // TriangleSelector.cpp:1154
            }
            // Capsule3D::is_mesh_point_inside  // TriangleSelector.cpp:2112-2127
            CursorShape::Capsule3D { first_center, second_center } => {
                let transformed_point = if self.base.uniform_scaling { *point } else { transform_point_f32(&self.base.trafo, point) };
                let first_center_diff = first_center - transformed_point;
                let second_center_diff = second_center - transformed_point;
                if first_center_diff.norm_squared() < self.base.radius_sqr || second_center_diff.norm_squared() < self.base.radius_sqr {
                    return is_mesh_point_not_clipped(point, &self.base.clipping_plane);
                }
                // First, check if the point pt is laying between planes defined by first_center and second_center.
                // Then check if it is inside the cylinder between first_center and second_center.
                let centers_diff = second_center - first_center;
                if dotf(&first_center_diff, &centers_diff) <= 0.0
                    && dotf(&second_center_diff, &centers_diff) >= 0.0
                    && (crossf(&first_center_diff, &centers_diff).norm() / centers_diff.norm()) <= self.base.radius
                {
                    return is_mesh_point_not_clipped(point, &self.base.clipping_plane);
                }
                false
            }
            // Capsule2D::is_mesh_point_inside  // TriangleSelector.cpp:2130-2160
            CursorShape::Capsule2D { first_center, second_center } => {
                let transformed_point = if self.base.uniform_scaling { *point } else { transform_point_f32(&self.base.trafo, point) };
                let first_center_diff = first_center - transformed_point;
                let first_center_diff_projected = first_center_diff - dotf(&first_center_diff, &self.base.dir) * self.base.dir;
                if first_center_diff_projected.norm_squared() < self.base.radius_sqr {
                    return is_mesh_point_not_clipped(point, &self.base.clipping_plane);
                }
                let second_center_diff = second_center - transformed_point;
                let second_center_diff_projected = second_center_diff - dotf(&second_center_diff, &self.base.dir) * self.base.dir;
                if second_center_diff_projected.norm_squared() < self.base.radius_sqr {
                    return is_mesh_point_not_clipped(point, &self.base.clipping_plane);
                }
                let centers_diff = second_center - first_center;
                let centers_diff_projected = centers_diff - dotf(&centers_diff, &self.base.dir) * self.base.dir;
                // First, check if the point is laying between first_center and second_center.
                if dotf(&first_center_diff_projected, &centers_diff_projected) <= 0.0 && dotf(&second_center_diff_projected, &centers_diff_projected) >= 0.0 {
                    // Vector in the direction of line |AD| of the rectangle that intersects the circle with the center in first_center.
                    let rectangle_da_dir = crossf(&centers_diff, &self.base.dir);
                    // Vector pointing from first_center to the point 'A' of the rectangle.
                    let first_center_rectangle_a_diff = rectangle_da_dir.normalize() * self.base.radius;
                    let rectangle_a = first_center - first_center_rectangle_a_diff;
                    let rectangle_d = first_center + first_center_rectangle_a_diff;
                    // Now check if the point is laying inside the rectangle between circles with centers in first_center and second_center.
                    if dotf(&(rectangle_a - transformed_point), &rectangle_da_dir) <= 0.0 && dotf(&(rectangle_d - transformed_point), &rectangle_da_dir) >= 0.0 {
                        return is_mesh_point_not_clipped(point, &self.base.clipping_plane);
                    }
                }
                false
            }
        }
    }

    // virtual bool is_edge_inside_cursor(const Triangle &tr, const std::vector<Vertex> &vertices) const — dispatch
    fn is_edge_inside_cursor(&self, tr: &Triangle, vertices: &[Vertex]) -> bool {
        match &self.shape {
            // Sphere::is_edge_inside_cursor  // TriangleSelector.cpp:1084-1100
            CursorShape::Sphere { center } => {
                let mut pts = [Vec3f::zeros(); 3];
                for i in 0..3usize {
                    pts[i] = vertices[tr.verts_idxs[i] as usize].v;
                    if !self.base.uniform_scaling {
                        pts[i] = transform_point_f32(&self.base.trafo, &pts[i]);
                    }
                }
                for side in 0..3usize {
                    let edge_a = pts[side];
                    let edge_b = pts[if side < 2 { side + 1 } else { 0 }];
                    if test_line_inside_sphere(&edge_a, &edge_b, center, self.base.radius) {
                        return true;
                    }
                }
                false
            }
            // Circle::is_edge_inside_cursor  // TriangleSelector.cpp:1103-1127
            CursorShape::Circle { center } => {
                let mut pts = [Vec3f::zeros(); 3];
                for i in 0..3usize {
                    pts[i] = vertices[tr.verts_idxs[i] as usize].v;
                    if !self.base.uniform_scaling {
                        pts[i] = transform_point_f32(&self.base.trafo, &pts[i]);
                    }
                }
                let p = center;
                for side in 0..3usize {
                    let a = pts[side];
                    let b = pts[if side < 2 { side + 1 } else { 0 }];
                    let s = (b - a).normalize();
                    let t = dotf(&(p - a), &s);
                    let vector = a + t * s - p;
                    // vector is 3D vector from center to the intersection. What we want to
                    // measure is length of its projection onto plane perpendicular to dir.
                    let dist_sqr = vector.norm_squared() - dotf(&vector, &self.base.dir).powf(2.0);
                    if dist_sqr < self.base.radius_sqr && t >= 0.0 && t <= (b - a).norm() {
                        return true;
                    }
                }
                false
            }
            // HeightRange::is_edge_inside_cursor  // TriangleSelector.cpp:1157-1169
            CursorShape::HeightRange { m_z_world, m_height, .. } => {
                let top_z = m_z_world + m_height + EPSILON as f32; // TriangleSelector.cpp:1159
                let bot_z = m_z_world - EPSILON as f32; // TriangleSelector.cpp:1160
                let mut pts = [Vec3f::zeros(); 3];
                for i in 0..3usize {
                    pts[i] = vertices[tr.verts_idxs[i] as usize].v;
                    pts[i] = transform_point_f32(&self.base.trafo, &pts[i]);
                }
                // TriangleSelector.cpp:1167-1168
                !((pts[0].z < bot_z && pts[1].z < bot_z && pts[2].z < bot_z) || (pts[0].z > top_z && pts[1].z > top_z && pts[2].z > top_z))
            }
            // Capsule3D::is_edge_inside_cursor  // TriangleSelector.cpp:2214-2231
            CursorShape::Capsule3D { first_center, second_center } => {
                let mut pts = [Vec3f::zeros(); 3];
                for i in 0..3usize {
                    pts[i] = vertices[tr.verts_idxs[i] as usize].v;
                    if !self.base.uniform_scaling {
                        pts[i] = transform_point_f32(&self.base.trafo, &pts[i]);
                    }
                }
                for side in 0..3usize {
                    let edge_a = pts[side];
                    let edge_b = pts[if side < 2 { side + 1 } else { 0 }];
                    if test_line_inside_capsule(&edge_a, &edge_b, first_center, second_center, self.base.radius) {
                        return true;
                    }
                }
                false
            }
            // Capsule2D::is_edge_inside_cursor  // TriangleSelector.cpp:2234-2286
            CursorShape::Capsule2D { first_center, second_center } => {
                let mut pts = [Vec3f::zeros(); 3];
                for i in 0..3usize {
                    pts[i] = vertices[tr.verts_idxs[i] as usize].v;
                    if !self.base.uniform_scaling {
                        pts[i] = transform_point_f32(&self.base.trafo, &pts[i]);
                    }
                }
                let centers_diff = second_center - first_center;
                // Vector in the direction of line |AD| of the rectangle that intersects the circle with the center in first_center.
                let rectangle_da_dir = crossf(&centers_diff, &self.base.dir);
                // Vector pointing from first_center to the point 'A' of the rectangle.
                let first_center_rectangle_a_diff = rectangle_da_dir.normalize() * self.base.radius;
                let rectangle_a = first_center - first_center_rectangle_a_diff;
                let rectangle_d = first_center + first_center_rectangle_a_diff;

                let edge_inside_rectangle = |edge_a: &Vec3f, edge_b: &Vec3f, plane_origin: &Vec3f, plane_normal: &Vec3f| -> bool {
                    let mut intersection = Vec3f::new(-1.0, -1.0, -1.0); // TriangleSelector.cpp:2252
                    if line_plane_intersection(edge_a, edge_b, plane_origin, plane_normal, &mut intersection) {
                        // Now check if the intersection point is inside the rectangle. That means it is between 'first_center' and 'second_center', resp. between 'A' and 'B'.
                        if dotf(first_center, &centers_diff) <= dotf(&intersection, &centers_diff) && dotf(&intersection, &centers_diff) <= dotf(second_center, &centers_diff) {
                            return true;
                        }
                    }
                    false
                };

                for side in 0..3usize {
                    let edge_a = pts[side];
                    let edge_b = pts[if side < 2 { side + 1 } else { 0 }];
                    let edge_dir = edge_b - edge_a;
                    let edge_dir_n = edge_dir.normalize();

                    let t1 = dotf(&(first_center - edge_a), &edge_dir_n);
                    let t2 = dotf(&(second_center - edge_a), &edge_dir_n);
                    let vector1 = edge_a + t1 * edge_dir_n - first_center;
                    let vector2 = edge_a + t2 * edge_dir_n - second_center;

                    // Vectors vector1 and vector2 are 3D vector from centers to the intersections.
                    let dist = vector1.norm_squared() - dotf(&vector1, &self.base.dir).powf(2.0);
                    if dist < self.base.radius_sqr && t1 >= 0.0 && t1 <= edge_dir.norm() {
                        return true;
                    }

                    let dist = vector2.norm_squared() - dotf(&vector2, &self.base.dir).powf(2.0);
                    if dist < self.base.radius_sqr && t2 >= 0.0 && t2 <= edge_dir.norm() {
                        return true;
                    }

                    // Check if the edge is passing through the rectangle between first_center and second_center.
                    if edge_inside_rectangle(&edge_a, &edge_b, &rectangle_a, &(rectangle_d - rectangle_a)) || edge_inside_rectangle(&edge_a, &edge_b, &rectangle_d, &(rectangle_a - rectangle_d)) {
                        return true;
                    }
                }
                false
            }
        }
    }

    // virtual bool is_facet_visible(int facet_idx, const std::vector<Vec3f> &face_normals) const — dispatch
    fn is_facet_visible(&self, facet_idx: i32, face_normals: &[Vec3f]) -> bool {
        match &self.shape {
            // Sphere / HeightRange / Capsule3D: return true (TriangleSelector.hpp:139,170,190)
            CursorShape::Sphere { .. } | CursorShape::HeightRange { .. } | CursorShape::Capsule3D { .. } => true,
            // Circle / Capsule2D: TriangleSelector::Cursor::is_facet_visible(*this, facet_idx, face_normals)
            CursorShape::Circle { .. } | CursorShape::Capsule2D { .. } => Self::is_facet_visible_static(self, facet_idx, face_normals),
        }
    }

    // Determine whether this facet is potentially visible (still can be obscured).
    // static bool is_facet_visible(const Cursor &cursor, int facet_idx, const std::vector<Vec3f> &face_normals)
    // TriangleSelector.cpp:1063-1070
    fn is_facet_visible_static(cursor: &Cursor, facet_idx: i32, face_normals: &[Vec3f]) -> bool {
        debug_assert!(facet_idx < face_normals.len() as i32); // TriangleSelector.cpp:1065
        let mut n = face_normals[facet_idx as usize]; // TriangleSelector.cpp:1066
        if !cursor.base.uniform_scaling {
            // TriangleSelector.cpp:1377 — n = cursor.trafo_normal * n; (a Transform3f with a zero translation)
            n = if eigen_fp() { transform_point_f32(&cursor.base.trafo_normal, &n) } else { linear_f32(&cursor.base.trafo_normal) * n };
        }
        dotf(&n, &cursor.base.dir) < 0.0 // TriangleSelector.cpp:1378
    }
}

/// `class TriangleSelector::SinglePointCursor` (TriangleSelector.hpp:78-104). Its data and virtual
/// methods are those of [`Cursor`] (see [`CursorShape`]); this type carries its static factories.
pub struct SinglePointCursor;

impl SinglePointCursor {
    /// TriangleSelector.hpp:86-93 `static std::unique_ptr<Cursor> cursor_factory(const Vec3f &center, const Vec3f &camera_pos, const float cursor_radius, const CursorType cursor_type, const Transform3d &trafo_matrix, const ClippingPlane &clipping_plane)`
    pub fn cursor_factory(center: &Vec3f, camera_pos: &Vec3f, cursor_radius: f32, cursor_type: CursorType, trafo_matrix: &Transform3d, clipping_plane: &ClippingPlane) -> Cursor {
        debug_assert!(cursor_type == CursorType::Circle || cursor_type == CursorType::Sphere); // TriangleSelector.hpp:88
        if cursor_type == CursorType::Sphere {
            Cursor::new_sphere(center, camera_pos, cursor_radius, trafo_matrix, clipping_plane) // TriangleSelector.hpp:90
        } else {
            Cursor::new_circle(center, camera_pos, cursor_radius, trafo_matrix, clipping_plane) // TriangleSelector.hpp:92
        }
    }

    /// TriangleSelector.hpp:95-98 `static std::unique_ptr<Cursor> cursor_factory(float z_world, const Vec3f& camera_pos, const float height, const Transform3d& trafo_matrix, const ClippingPlane& clipping_plane)`:
    /// the second overload of `cursor_factory` (Rust has no overloading).
    pub fn cursor_factory_height_range(z_world: f32, camera_pos: &Vec3f, height: f32, trafo_matrix: &Transform3d, clipping_plane: &ClippingPlane) -> Cursor {
        Cursor::new_height_range(z_world, camera_pos, height, trafo_matrix, clipping_plane) // TriangleSelector.hpp:97
    }
}

/// `class TriangleSelector::DoublePointCursor` (TriangleSelector.hpp:106-128). Its data and virtual
/// methods are those of [`Cursor`] (see [`CursorShape`]); this type carries its static factory.
pub struct DoublePointCursor;

impl DoublePointCursor {
    /// TriangleSelector.hpp:114-121 `static std::unique_ptr<Cursor> cursor_factory(const Vec3f &first_center, const Vec3f &second_center, const Vec3f &camera_pos, const float cursor_radius, const CursorType cursor_type, const Transform3d &trafo_matrix, const ClippingPlane &clipping_plane)`
    pub fn cursor_factory(first_center: &Vec3f, second_center: &Vec3f, camera_pos: &Vec3f, cursor_radius: f32, cursor_type: CursorType, trafo_matrix: &Transform3d, clipping_plane: &ClippingPlane) -> Cursor {
        debug_assert!(cursor_type == CursorType::Circle || cursor_type == CursorType::Sphere); // TriangleSelector.hpp:116
        if cursor_type == CursorType::Sphere {
            Cursor::new_capsule3d(first_center, second_center, camera_pos, cursor_radius, trafo_matrix, clipping_plane) // TriangleSelector.hpp:118
        } else {
            Cursor::new_capsule2d(first_center, second_center, camera_pos, cursor_radius, trafo_matrix, clipping_plane) // TriangleSelector.hpp:120
        }
    }
}

// ============================================================================
// TriangleSelector.cpp free functions
// ============================================================================

/// Gate `TS_EIGEN_FP` (faithful): the float arithmetic of this file as the C++ build evaluates it
/// (clang `-ffp-contract=on`, Eigen 5): `Vec3f::dot` is `fma(a.z, b.z, a.x * b.x + a.y * b.y)`,
/// `Vec3f::cross` fuses each component, `Transform3f * Vec3f` is `fma(m2, v2, fma(m1, v1, m0 * v0))
/// + m3` per row, and each `a * b - c * d` / `m + t * n` of one expression is fused. Off: the
/// earlier nalgebra arithmetic, unfused. The two differ only in the last bit, which decides a test
/// only for a point on a boundary (a triangle edge, a cursor rim, the edge-length floor). Not
/// covered: the normal matrix of select_patch() and seed_fill_select_triangles() (Eigen's 3x3
/// inverse and `Matrix3f * Vec3f`) and the cursor's `trafo_normal` inverse, whose evaluation order
/// in the C++ build is not measured.
#[inline]
fn eigen_fp() -> bool {
    crate::faithful_gate("TS_EIGEN_FP")
}

/// `Vec3f::dot` as the C++ build evaluates it (R880)
#[inline]
fn eigen_dot_f32(a: &Vec3f, b: &Vec3f) -> f32 {
    a.z.mul_add(b.z, a.x * b.x + a.y * b.y)
}

// TriangleSelector.cpp:224-243
#[inline]
fn is_point_inside_triangle(pt: &Vec3f, p1: &Vec3f, p2: &Vec3f, p3: &Vec3f) -> bool {
    // Real-time collision detection, Ericson, Chapter 3.4
    let barycentric = || -> Vec3f {
        let v = [p2 - p1, p3 - p1, pt - p1]; // TriangleSelector.cpp:228
        if eigen_fp() {
            let d00 = eigen_dot_f32(&v[0], &v[0]); // TriangleSelector.cpp:229
            let d01 = eigen_dot_f32(&v[0], &v[1]); // TriangleSelector.cpp:230
            let d11 = eigen_dot_f32(&v[1], &v[1]); // TriangleSelector.cpp:231
            let d20 = eigen_dot_f32(&v[2], &v[0]); // TriangleSelector.cpp:232
            let d21 = eigen_dot_f32(&v[2], &v[1]); // TriangleSelector.cpp:233
            let denom = d00.mul_add(d11, -(d01 * d01)); // TriangleSelector.cpp:234

            // TriangleSelector.cpp:236
            let mut barycentric_cords = Vec3f::new(1.0, d11.mul_add(d20, -(d01 * d21)) / denom, d00.mul_add(d21, -(d01 * d20)) / denom);
            barycentric_cords.x = barycentric_cords.x - barycentric_cords.y - barycentric_cords.z; // TriangleSelector.cpp:237
            return barycentric_cords; // TriangleSelector.cpp:238
        }
        // TS_EIGEN_FP=0: the earlier arithmetic.
        let d00 = v[0].dot(&v[0]);
        let d01 = v[0].dot(&v[1]);
        let d11 = v[1].dot(&v[1]);
        let d20 = v[2].dot(&v[0]);
        let d21 = v[2].dot(&v[1]);
        let denom = d00 * d11 - d01 * d01;

        let mut barycentric_cords = Vec3f::new(1.0, (d11 * d20 - d01 * d21) / denom, (d00 * d21 - d01 * d20) / denom);
        barycentric_cords.x = barycentric_cords.x - barycentric_cords.y - barycentric_cords.z;
        barycentric_cords
    };

    let barycentric_cords = barycentric(); // TriangleSelector.cpp:241
    // TriangleSelector.cpp:242 — std::all_of(begin, end, [](float cord){ return 0.f <= cord && cord <= 1.0; })
    barycentric_cords.iter().all(|&cord| 0.0 <= cord && cord as f64 <= 1.0)
}

// Returns true if clipping plane is not active or if the point not clipped by clipping plane.
// TriangleSelector.cpp:2084-2087
#[inline]
fn is_mesh_point_not_clipped(point: &Vec3f, clipping_plane: &ClippingPlane) -> bool {
    !clipping_plane.is_active() || !clipping_plane.is_mesh_point_clipped(point)
}

// p1, p2, p3 are in mesh coords!  // TriangleSelector.cpp:2163-2182
fn is_circle_pointer_inside_triangle(p1_: &Vec3f, p2_: &Vec3f, p3_: &Vec3f, center: &Vec3f, dir: &Vec3f, uniform_scaling: bool, trafo: &Transform3f) -> bool {
    let q1 = center + dir; // TriangleSelector.cpp:2164
    let q2 = center - dir; // TriangleSelector.cpp:2165

    // TriangleSelector.cpp:2167-2170
    let signed_volume_sign = |a: &Vec3f, b: &Vec3f, c: &Vec3f, d: &Vec3f| -> bool { dotf(&crossf(&(b - a), &(c - a)), &(d - a)) > 0.0 };

    // In case the object is non-uniformly scaled, do the check in world coords.
    let p1 = if uniform_scaling { *p1_ } else { transform_point_f32(trafo, p1_) }; // TriangleSelector.cpp:2173
    let p2 = if uniform_scaling { *p2_ } else { transform_point_f32(trafo, p2_) }; // TriangleSelector.cpp:2174
    let p3 = if uniform_scaling { *p3_ } else { transform_point_f32(trafo, p3_) }; // TriangleSelector.cpp:2175

    if signed_volume_sign(&q1, &p1, &p2, &p3) == signed_volume_sign(&q2, &p1, &p2, &p3) {
        // TriangleSelector.cpp:2177
        return false;
    }

    let pos = signed_volume_sign(&q1, &q2, &p1, &p2); // TriangleSelector.cpp:2180
    signed_volume_sign(&q1, &q2, &p2, &p3) == pos && signed_volume_sign(&q1, &q2, &p3, &p1) == pos // TriangleSelector.cpp:2181
}

// TriangleSelector.cpp:2197-2212
fn line_plane_intersection(line_a: &Vec3f, line_b: &Vec3f, plane_origin: &Vec3f, plane_normal: &Vec3f, out_intersection: &mut Vec3f) -> bool {
    let line_dir = line_b - line_a; // TriangleSelector.cpp:2199
    let t_denominator = dotf(plane_normal, &line_dir); // TriangleSelector.cpp:2546
    if t_denominator == 0.0 {
        // TriangleSelector.cpp:2201
        return false;
    }

    // Compute 'd' in plane equation by using some point (origin) on the plane
    let plane_d = dotf(plane_normal, plane_origin); // TriangleSelector.cpp:2551
    let t = (plane_d - dotf(plane_normal, line_a)) / t_denominator; // TriangleSelector.cpp:2552
    if t >= 0.0 && t <= 1.0 {
        *out_intersection = line_a + t * line_dir; // TriangleSelector.cpp:2207
        return true; // TriangleSelector.cpp:2208
    }

    false // TriangleSelector.cpp:2211
}

// ============================================================================
// TriangleSelector — the main class (TriangleSelector.hpp:16-437)
// ============================================================================

/// `class TriangleSelector` (TriangleSelector.hpp:16)
pub struct TriangleSelector {
    // Lists of vertices and triangles, both original and new  // TriangleSelector.hpp:366-371
    m_vertices: Vec<Vertex>,
    m_triangles: Vec<Triangle>,
    m_mesh: TriangleMesh,
    m_neighbors: Vec<Vec3i>,
    m_face_normals: Vec<Vec3f>,

    // BBS  // TriangleSelector.hpp:374
    m_edge_limit: f32,

    // Number of invalid triangles (to trigger garbage collection).  // TriangleSelector.hpp:377
    m_invalid_triangles: i32,

    // Limiting length of triangle side (squared).  // TriangleSelector.hpp:380
    m_edge_limit_sqr: f32,

    // Number of original vertices and triangles.  // TriangleSelector.hpp:383-384
    m_orig_size_vertices: i32,
    m_orig_size_indices: i32,

    m_cursor: Option<Cursor>, // TriangleSelector.hpp:386
    // Zero indicates an uninitialized state.  // TriangleSelector.hpp:388
    m_old_cursor_radius_sqr: f32,

    m_free_triangles_head: i32, // TriangleSelector.hpp:435
    m_free_vertices_head: i32, // TriangleSelector.hpp:436
}

// ============================================================================
// TriangleSelector.hpp:268-312, 480-496 — subdivision and region-paint types
// ============================================================================

/// `struct TriangleSelector::FacetSubdivisionMeasurement` (TriangleSelector.hpp:268-272)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FacetSubdivisionMeasurement {
    pub dominant_state: EnforcerBlockerType,
    pub surface_area: f64,
    pub error_area: f64,
}

impl Default for FacetSubdivisionMeasurement {
    fn default() -> Self {
        // TriangleSelector.hpp:269-271 — { EnforcerBlockerType::NONE }, { 0.0 }, { 0.0 }
        FacetSubdivisionMeasurement { dominant_state: EnforcerBlockerType::NONE, surface_area: 0.0, error_area: 0.0 }
    }
}

/// `struct TriangleSelector::FacetSubdivisionResult` (TriangleSelector.hpp:273-280)
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct FacetSubdivisionResult {
    pub surface_area: f64,
    pub error_area: f64,
    pub nodes_created: usize,
    pub node_budget_exhausted: bool,
    pub depth_limit_reached: bool,
    pub canceled: bool,
}

/// `using FacetSubdivisionEvaluator = std::function<FacetSubdivisionMeasurement(int, const std::array<Vec3f, 3>&)>;`
/// (TriangleSelector.hpp:281-282)
pub type FacetSubdivisionEvaluator<'a> = dyn FnMut(i32, &[Vec3f; 3]) -> FacetSubdivisionMeasurement + 'a;
/// `using FacetSubdivisionCancelCallback = std::function<bool()>;` (TriangleSelector.hpp:283-284):
/// returns true to request early termination of the subdivision loop. An empty `std::function`
/// is `None` where it is taken.
pub type FacetSubdivisionCancelCallback<'a> = dyn FnMut() -> bool + 'a;

/// `enum class TriangleSelector::RegionCoverage` (TriangleSelector.hpp:298-303): how much of a
/// destination triangle a source paint region covers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RegionCoverage {
    /// no overlap at all: leave the triangle alone
    None,
    /// the region boundary crosses the triangle: refine
    Partial,
    /// the region covers the triangle: paint it as a whole
    Full,
}

/// `using RegionCoverageEvaluator = std::function<RegionCoverage(const std::array<Vec3f, 3> &)>;`
/// (TriangleSelector.hpp:304-305). An empty `std::function` is `None` where it is taken.
pub type RegionCoverageEvaluator<'a> = dyn FnMut(&[Vec3f; 3]) -> RegionCoverage + 'a;

/// `struct TriangleSelector::RegionPaintResult` (TriangleSelector.hpp:307-312)
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct RegionPaintResult {
    pub nodes_created: usize,
    pub facets_touched: usize,
    pub node_budget_exhausted: bool,
    pub canceled: bool,
}

/// `struct TriangleSelector::RegionPaintContext` (TriangleSelector.hpp:480-493): everything a
/// paint_region() walk carries from the top down, bundled so the recursion keeps a short signature
/// and so the cancel poll can share one counter with the outer breadth-first walk.
pub struct RegionPaintContext<'a> {
    coverage: &'a mut RegionCoverageEvaluator<'a>,
    cancel: Option<&'a mut FacetSubdivisionCancelCallback<'a>>,
    trafo: Option<&'a Transform3f>,
    new_state: EnforcerBlockerType,
    max_depth: i32,
    node_budget: usize,
    steps_since_cancel_check: usize, // TriangleSelector.hpp:491 — { 0 }
    result: RegionPaintResult,
}

/// TriangleSelector.cpp:1153-1157: cancel is polled once per this many visited nodes: the callback
/// reaches a mutex and an atomic through two std::function hops, so polling per node would cost
/// more than the coverage tests it is guarding, while polling only per seed leaves a whole
/// subdivision tree of AABB traversals between two checks.
pub const REGION_PAINT_CANCEL_POLL_INTERVAL: usize = 256;

impl TriangleSelector {
    // ---- accessors (TriangleSelector.hpp:220-223) ----
    pub fn get_orig_size_vertices(&self) -> i32 {
        self.m_orig_size_vertices
    }
    pub fn get_triangles(&self) -> &Vec<Triangle> {
        &self.m_triangles
    }
    pub fn get_vertices(&self) -> &Vec<Vertex> {
        &self.m_vertices
    }
    pub fn get_neighbors(&self) -> &Vec<Vec3i> {
        &self.m_neighbors
    }
    /// Read-only view of the C++ member `m_edge_limit_sqr` (TriangleSelector.hpp:466), for the
    /// parity test (the C++ generator reads the member directly).
    #[doc(hidden)]
    pub fn edge_limit_sqr(&self) -> f32 {
        self.m_edge_limit_sqr
    }
    /// Read-only view of the C++ member `m_old_cursor_radius_sqr` (TriangleSelector.hpp:474), for
    /// the parity test.
    #[doc(hidden)]
    pub fn old_cursor_radius_sqr(&self) -> f32 {
        self.m_old_cursor_radius_sqr
    }

    // Provide the mesh's indexed_triangle_set (C++ `m_mesh.its`).
    fn its(&self) -> indexed_triangle_set {
        // The C++ class stores a reference to a TriangleMesh and reads m_mesh.its.
        // The crate's TriangleMesh stores vertices/indices separately; convert on demand
        // for the few reset/deserialize sites that touch m_mesh.its directly.
        let mut its = indexed_triangle_set::default();
        its.vertices.reserve(self.m_mesh.vertices().len());
        for p in self.m_mesh.vertices() {
            its.vertices.push(StlVertex::new(p.x as f32, p.y as f32, p.z as f32));
        }
        its.indices.reserve(self.m_mesh.indices().len());
        for tri in self.m_mesh.indices() {
            its.indices.push(StlTriangleVertexIndices::new(tri.indices[0] as i32, tri.indices[1] as i32, tri.indices[2] as i32));
        }
        its
    }

    // ============================================================================
    // TriangleSelector.cpp:120-156 — verification helpers (NDEBUG only)
    // ============================================================================

    // bool verify_triangle_midpoints(const Triangle &tr) const  // TriangleSelector.cpp:120-135
    #[cfg(debug_assertions)]
    fn verify_triangle_midpoints(&self, tr: &Triangle) -> bool {
        for i in 0..3usize {
            let v1 = tr.verts_idxs[i]; // TriangleSelector.cpp:123
            let v2 = tr.verts_idxs[next_idx_modulo(i, 3)]; // TriangleSelector.cpp:124
            let vmid = tr.triangle_midpoint(self, v1, v2); // TriangleSelector.cpp:125
            debug_assert!(vmid >= -1); // TriangleSelector.cpp:126
            if vmid != -1 {
                let c1 = 0.5 * (self.m_vertices[v1 as usize].v + self.m_vertices[v2 as usize].v); // TriangleSelector.cpp:128
                let c2 = self.m_vertices[vmid as usize].v; // TriangleSelector.cpp:129
                let d = (c2 - c1).norm(); // TriangleSelector.cpp:130
                debug_assert!((d as f64).abs() < EPSILON); // TriangleSelector.cpp:131
            }
        }
        true
    }

    // bool verify_triangle_neighbors(const Triangle &tr, const Vec3i &neighbors) const  // TriangleSelector.cpp:174-193
    //
    // The engine compiles TriangleSelector.cpp with NDEBUG, where this check and the asserts that
    // call it are off. The pin's own code breaks it on ordinary input: set_facets_with_subdivision()
    // splits a queued candidate with the neighbours it was queued with, which no longer name the
    // neighbour's children once that neighbour has been split in the meantime (the C++ asserts fire
    // there too, built without NDEBUG). In a debug build it is therefore checked only with the
    // diagnostic switch `TS_VERIFY` set; it never changes what the selector does.
    #[cfg(debug_assertions)]
    fn verify_triangle_neighbors(&self, tr: &Triangle, neighbors: &Vec3i) -> bool {
        if !crate::probe_enabled("TS_VERIFY") {
            return true;
        }
        debug_assert!(neighbors[0] >= -1); // TriangleSelector.cpp:139
        debug_assert!(neighbors[1] >= -1); // TriangleSelector.cpp:140
        debug_assert!(neighbors[2] >= -1); // TriangleSelector.cpp:141
        debug_assert!(self.verify_triangle_midpoints(tr)); // TriangleSelector.cpp:142

        for i in 0..3usize {
            if neighbors[i] != -1 {
                let tr2 = &self.m_triangles[neighbors[i] as usize]; // TriangleSelector.cpp:146
                debug_assert!(self.verify_triangle_midpoints(tr2)); // TriangleSelector.cpp:147
                let v1 = tr.verts_idxs[i]; // TriangleSelector.cpp:148
                let v2 = tr.verts_idxs[next_idx_modulo(i, 3)]; // TriangleSelector.cpp:149
                debug_assert!(tr2.verts_idxs[0] == v1 || tr2.verts_idxs[1] == v1 || tr2.verts_idxs[2] == v1); // TriangleSelector.cpp:150
                let j = if tr2.verts_idxs[0] == v1 { 0 } else if tr2.verts_idxs[1] == v1 { 1 } else { 2 }; // TriangleSelector.cpp:151
                debug_assert!(tr2.verts_idxs[j] == v1); // TriangleSelector.cpp:152
                debug_assert!(tr2.verts_idxs[prev_idx_modulo(j, 3)] == v2); // TriangleSelector.cpp:153
            }
        }
        true
    }

    // No-op verifier in release builds (NDEBUG).
    #[cfg(not(debug_assertions))]
    #[inline]
    fn verify_triangle_neighbors(&self, _tr: &Triangle, _neighbors: &Vec3i) -> bool {
        true
    }
}

impl TriangleSelector {
    // ============================================================================
    // TriangleSelector.cpp:208-250 — select_unsplit_triangle
    // ============================================================================

    // [[nodiscard]] int select_unsplit_triangle(const Vec3f &hit, int facet_idx, const Vec3i &neighbors) const
    // TriangleSelector.cpp:208-239
    pub fn select_unsplit_triangle_n(&self, hit: &Vec3f, facet_idx: i32, neighbors: &Vec3i) -> i32 {
        debug_assert!(facet_idx < self.m_triangles.len() as i32); // TriangleSelector.cpp:210
        let tr = &self.m_triangles[facet_idx as usize]; // TriangleSelector.cpp:211
        if !tr.valid() {
            return -1; // TriangleSelector.cpp:213
        }

        if !tr.is_split() {
            // TriangleSelector.cpp:216
            let t_vert = self.m_triangles[facet_idx as usize].verts_idxs;
            if is_point_inside_triangle(hit, &self.m_vertices[t_vert[0] as usize].v, &self.m_vertices[t_vert[1] as usize].v, &self.m_vertices[t_vert[2] as usize].v) {
                return facet_idx; // TriangleSelector.cpp:217
            }
            return -1; // TriangleSelector.cpp:219
        }

        debug_assert!(self.verify_triangle_neighbors(tr, neighbors)); // TriangleSelector.cpp:222

        let num_of_children = tr.number_of_split_sides() + 1; // TriangleSelector.cpp:224
        if num_of_children != 1 {
            for i in 0..num_of_children as usize {
                debug_assert!(i < tr.children.len()); // TriangleSelector.cpp:227
                debug_assert!(tr.children[i] < self.m_triangles.len() as i32); // TriangleSelector.cpp:228
                // Recursion, deep first search over the children of this triangle.
                let t_vert = self.m_triangles[tr.children[i] as usize].verts_idxs; // TriangleSelector.cpp:232
                if is_point_inside_triangle(hit, &self.m_vertices[t_vert[0] as usize].v, &self.m_vertices[t_vert[1] as usize].v, &self.m_vertices[t_vert[2] as usize].v) {
                    // TriangleSelector.cpp:234
                    return self.select_unsplit_triangle_n(hit, tr.children[i], &self.child_neighbors(tr, neighbors, i as i32));
                }
            }
        }

        -1 // TriangleSelector.cpp:238
    }

    // [[nodiscard]] int select_unsplit_triangle(const Vec3f &hit, int facet_idx) const
    // TriangleSelector.cpp:241-250
    pub fn select_unsplit_triangle(&self, hit: &Vec3f, facet_idx: i32) -> i32 {
        debug_assert!(facet_idx < self.m_triangles.len() as i32); // TriangleSelector.cpp:243
        if !self.m_triangles[facet_idx as usize].valid() {
            return -1; // TriangleSelector.cpp:245
        }

        let neighbors = self.m_neighbors[facet_idx as usize]; // TriangleSelector.cpp:247
        debug_assert!(self.verify_triangle_neighbors(&self.m_triangles[facet_idx as usize], &neighbors)); // TriangleSelector.cpp:248
        self.select_unsplit_triangle_n(hit, facet_idx, &neighbors) // TriangleSelector.cpp:249
    }

    // ============================================================================
    // TriangleSelector.cpp:252-326 — select_patch
    // ============================================================================

    // void select_patch(int facet_start, std::unique_ptr<Cursor> &&cursor, EnforcerBlockerType new_state, const Transform3d& trafo_no_translate, bool triangle_splitting, float highlight_by_angle_deg)
    // TriangleSelector.cpp:252-326
    pub fn select_patch(&mut self, facet_start: i32, cursor: Cursor, new_state: EnforcerBlockerType, trafo_no_translate: &Transform3d, triangle_splitting: bool, highlight_by_angle_deg: f32) {
        debug_assert!(facet_start < self.m_orig_size_indices); // TriangleSelector.cpp:254

        // Save current cursor center, squared radius and camera direction.
        self.m_cursor = Some(cursor); // TriangleSelector.cpp:258

        // In case user changed cursor size since last time, update triangle edge limit.
        // TriangleSelector.cpp:263
        let cursor_radius_sqr = self.m_cursor.as_ref().unwrap().base.radius_sqr;
        if self.m_old_cursor_radius_sqr != cursor_radius_sqr {
            // BBS: improve details for large cursor radius
            // TriangleSelector.cpp:265 — dynamic_cast<HeightRange*>
            let is_hr = matches!(self.m_cursor.as_ref().unwrap().shape, CursorShape::HeightRange { .. });
            if !is_hr {
                self.set_edge_limit((cursor_radius_sqr.sqrt() / 5.0).min(0.2)); // TriangleSelector.cpp:267
                self.m_old_cursor_radius_sqr = cursor_radius_sqr; // TriangleSelector.cpp:268
            } else {
                self.set_edge_limit(0.1); // TriangleSelector.cpp:271
                self.m_old_cursor_radius_sqr = 0.1; // TriangleSelector.cpp:272
            }
        }

        // TriangleSelector.cpp:313 — const float highlight_angle_limit = -cos(Geometry::deg2rad(highlight_by_angle_deg));
        let highlight_angle_limit = if float_angle_limits() { -deg2rad_f32(highlight_by_angle_deg).cos() } else { -(deg2rad(highlight_by_angle_deg as f64).cos()) as f32 };

        // BBS
        let mut start_facets: Vec<i32> = Vec::new(); // TriangleSelector.cpp:279
        let is_hr = matches!(self.m_cursor.as_ref().unwrap().shape, CursorShape::HeightRange { .. }); // TriangleSelector.cpp:280
        if is_hr {
            for facet_id in 0..self.m_orig_size_indices {
                // TriangleSelector.cpp:282
                let tr = &self.m_triangles[facet_id as usize]; // TriangleSelector.cpp:283
                if self.m_cursor.as_ref().unwrap().is_edge_inside_cursor(tr, &self.m_vertices) {
                    // TriangleSelector.cpp:284
                    start_facets.push(facet_id); // TriangleSelector.cpp:285
                }
            }
        } else {
            start_facets.push(facet_start); // TriangleSelector.cpp:290
        }

        // Keep track of facets of the original mesh we already processed.
        let mut visited = vec![false; self.m_orig_size_indices as usize]; // TriangleSelector.cpp:294

        for i in 0..start_facets.len() {
            // TriangleSelector.cpp:296
            let start_facet_id = start_facets[i]; // TriangleSelector.cpp:297
            if visited[start_facet_id as usize] {
                continue; // TriangleSelector.cpp:299
            }

            // Now start with the facet the pointer points to and check all adjacent facets.
            let mut facets_to_check: Vec<i32> = Vec::with_capacity(16); // TriangleSelector.cpp:302-303
            facets_to_check.push(start_facet_id); // TriangleSelector.cpp:304

            // Breadth-first search around the hit point.
            let mut facet_idx = 0usize; // TriangleSelector.cpp:308
            while facet_idx < facets_to_check.len() {
                // TriangleSelector.cpp:309
                let facet = facets_to_check[facet_idx]; // TriangleSelector.cpp:310
                let facet_normal = self.m_face_normals[self.m_triangles[facet as usize].source_triangle as usize]; // TriangleSelector.cpp:311
                // Matrix3f normal_matrix = trafo_no_translate.matrix().block(0,0,3,3).inverse().transpose().cast<float>();
                // TriangleSelector.cpp:312
                let block = trafo_no_translate.fixed_view::<3, 3>(0, 0).into_owned();
                let inv = block.try_inverse().unwrap_or_else(nalgebra::Matrix3::<f64>::zeros);
                let normal_matrix: Matrix3f = inv.transpose().cast::<f32>();
                let world_normal_z = (normal_matrix * facet_normal).normalize().z; // TriangleSelector.cpp:313
                if !visited[facet as usize] && (highlight_by_angle_deg == 0.0 || world_normal_z < highlight_angle_limit) {
                    // TriangleSelector.cpp:314
                    if self.select_triangle(facet, new_state, triangle_splitting) {
                        // TriangleSelector.cpp:315
                        // add neighboring facets to list to be processed later
                        for k in 0..3usize {
                            let neighbor_idx = self.m_neighbors[facet as usize][k]; // TriangleSelector.cpp:317
                            if neighbor_idx >= 0 && self.m_cursor.as_ref().unwrap().is_facet_visible(neighbor_idx, &self.m_face_normals) {
                                // TriangleSelector.cpp:318
                                facets_to_check.push(neighbor_idx); // TriangleSelector.cpp:319
                            }
                        }
                    }
                }
                visited[facet as usize] = true; // TriangleSelector.cpp:322
                facet_idx += 1; // TriangleSelector.cpp:323
            }
        }
    }

    // ============================================================================
    // TriangleSelector.cpp:328-335 — is_facet_clipped
    // ============================================================================

    // bool is_facet_clipped(int facet_idx, const ClippingPlane &clp) const  // TriangleSelector.cpp:328-335
    fn is_facet_clipped(&self, facet_idx: i32, clp: &ClippingPlane) -> bool {
        for vert_idx in self.m_triangles[facet_idx as usize].verts_idxs {
            // TriangleSelector.cpp:330
            if clp.is_active() && clp.is_mesh_point_clipped(&self.m_vertices[vert_idx as usize].v) {
                return true; // TriangleSelector.cpp:332
            }
        }
        false // TriangleSelector.cpp:334
    }

    // ============================================================================
    // TriangleSelector.cpp:337-391 — seed_fill_select_triangles
    // ============================================================================

    // void seed_fill_select_triangles(const Vec3f &hit, int facet_start, const Transform3d& trafo_no_translate, const ClippingPlane &clp, float seed_fill_angle, float highlight_by_angle_deg, bool force_reselection)
    // TriangleSelector.cpp:337-391
    pub fn seed_fill_select_triangles(&mut self, hit: &Vec3f, facet_start: i32, trafo_no_translate: &Transform3d, clp: &ClippingPlane, seed_fill_angle: f32, highlight_by_angle_deg: f32, force_reselection: bool) {
        debug_assert!(facet_start < self.m_orig_size_indices); // TriangleSelector.cpp:341

        // Recompute seed fill only if the cursor is pointing on facet unselected by seed fill or a clipping plane is active.
        // TriangleSelector.cpp:344
        let start_facet_idx = self.select_unsplit_triangle(hit, facet_start);
        if start_facet_idx >= 0 && self.m_triangles[start_facet_idx as usize].is_selected_by_seed_fill() && !force_reselection && !clp.is_active() {
            return; // TriangleSelector.cpp:345
        }

        self.seed_fill_unselect_all_triangles(); // TriangleSelector.cpp:347

        let mut visited = vec![false; self.m_triangles.len()]; // TriangleSelector.cpp:349
        let mut facet_queue: VecDeque<i32> = VecDeque::new(); // TriangleSelector.cpp:350
        facet_queue.push_back(facet_start); // TriangleSelector.cpp:351

        // TriangleSelector.cpp:390 — const double facet_angle_limit = cos(Geometry::deg2rad(seed_fill_angle)) - EPSILON;
        let facet_angle_limit = if float_angle_limits() { deg2rad_f32(seed_fill_angle).cos() as f64 - EPSILON } else { deg2rad(seed_fill_angle as f64).cos() - EPSILON };
        // TriangleSelector.cpp:391 — const float highlight_angle_limit = -cos(Geometry::deg2rad(highlight_by_angle_deg));
        let highlight_angle_limit = if float_angle_limits() { -deg2rad_f32(highlight_by_angle_deg).cos() } else { -(deg2rad(highlight_by_angle_deg as f64).cos()) as f32 };

        // Depth-first traversal of neighbors of the face hit by the ray thrown from the mouse cursor.
        while let Some(current_facet) = facet_queue.pop_front() {
            // TriangleSelector.cpp:357-359
            let facet_normal = self.m_face_normals[self.m_triangles[current_facet as usize].source_triangle as usize]; // TriangleSelector.cpp:361
            // TriangleSelector.cpp:362
            let block = trafo_no_translate.fixed_view::<3, 3>(0, 0).into_owned();
            let inv = block.try_inverse().unwrap_or_else(nalgebra::Matrix3::<f64>::zeros);
            let normal_matrix: Matrix3f = inv.transpose().cast::<f32>();
            let world_normal_z = (normal_matrix * facet_normal).normalize().z; // TriangleSelector.cpp:363
            if !visited[current_facet as usize] && (highlight_by_angle_deg == 0.0 || world_normal_z < highlight_angle_limit) {
                // TriangleSelector.cpp:364
                if self.m_triangles[current_facet as usize].is_split() {
                    // TriangleSelector.cpp:365
                    let n = self.m_triangles[current_facet as usize].number_of_split_sides();
                    for split_triangle_idx in 0..=n {
                        // TriangleSelector.cpp:366
                        debug_assert!((split_triangle_idx as usize) < self.m_triangles[current_facet as usize].children.len()); // TriangleSelector.cpp:367
                        debug_assert!(self.m_triangles[current_facet as usize].children[split_triangle_idx as usize] < self.m_triangles.len() as i32); // TriangleSelector.cpp:368
                        let child = self.m_triangles[current_facet as usize].children[split_triangle_idx as usize]; // TriangleSelector.cpp:369
                        if !visited[child as usize] {
                            // Child triangle shares normal with its parent. Select it.
                            facet_queue.push_back(child); // TriangleSelector.cpp:371
                        }
                    }
                } else {
                    self.m_triangles[current_facet as usize].select_by_seed_fill(); // TriangleSelector.cpp:374
                }

                if current_facet < self.m_orig_size_indices {
                    // TriangleSelector.cpp:376
                    // Propagate over the original triangles.
                    for k in 0..3usize {
                        let neighbor_idx = self.m_neighbors[current_facet as usize][k]; // TriangleSelector.cpp:378
                        debug_assert!(neighbor_idx >= -1); // TriangleSelector.cpp:379
                        if neighbor_idx >= 0 && !visited[neighbor_idx as usize] && !self.is_facet_clipped(neighbor_idx, clp) {
                            // TriangleSelector.cpp:380
                            // Check if neighbour_facet_idx satisfies angle in seed_fill_angle.
                            let n1 = self.m_face_normals[self.m_triangles[neighbor_idx as usize].source_triangle as usize]; // TriangleSelector.cpp:382
                            let n2 = self.m_face_normals[self.m_triangles[current_facet as usize].source_triangle as usize]; // TriangleSelector.cpp:383
                            if (dotf(&n1, &n2).clamp(0.0, 1.0) as f64) >= facet_angle_limit {
                                // TriangleSelector.cpp:384
                                facet_queue.push_back(neighbor_idx); // TriangleSelector.cpp:385
                            }
                        }
                    }
                }
            }
            visited[current_facet as usize] = true; // TriangleSelector.cpp:389
        }
    }

    // ============================================================================
    // TriangleSelector.cpp:393-434 — precompute_all_neighbors[_recursive]
    // ============================================================================

    // void precompute_all_neighbors_recursive(...) const  // TriangleSelector.cpp:393-420
    pub fn precompute_all_neighbors_recursive(&self, facet_idx: i32, neighbors: &Vec3i, neighbors_propagated: &Vec3i, neighbors_out: &mut Vec<Vec3i>, neighbors_propagated_out: &mut Vec<Vec3i>) {
        debug_assert!(facet_idx < self.m_triangles.len() as i32); // TriangleSelector.cpp:395

        let tr = &self.m_triangles[facet_idx as usize]; // TriangleSelector.cpp:397
        if !tr.valid() {
            return; // TriangleSelector.cpp:399
        }

        neighbors_out[facet_idx as usize] = *neighbors; // TriangleSelector.cpp:401
        neighbors_propagated_out[facet_idx as usize] = *neighbors_propagated; // TriangleSelector.cpp:402
        if tr.is_split() {
            debug_assert!(self.verify_triangle_neighbors(tr, neighbors)); // TriangleSelector.cpp:404

            let num_of_children = tr.number_of_split_sides() + 1; // TriangleSelector.cpp:406
            if num_of_children != 1 {
                for i in 0..num_of_children {
                    debug_assert!(i < tr.children.len() as i32); // TriangleSelector.cpp:409
                    debug_assert!(tr.children[i as usize] < self.m_triangles.len() as i32); // TriangleSelector.cpp:410
                    // Recursion, deep first search over the children of this triangle.
                    let child_neighbors = self.child_neighbors(tr, neighbors, i); // TriangleSelector.cpp:413
                    let child = tr.children[i as usize];
                    let child_prop = self.child_neighbors_propagated(tr, neighbors_propagated, i, &child_neighbors);
                    self.precompute_all_neighbors_recursive(child, &child_neighbors, &child_prop, neighbors_out, neighbors_propagated_out); // TriangleSelector.cpp:414-416
                }
            }
        }
    }

    // std::pair<std::vector<Vec3i>, std::vector<Vec3i>> precompute_all_neighbors() const  // TriangleSelector.cpp:422-434
    pub fn precompute_all_neighbors(&self) -> (Vec<Vec3i>, Vec<Vec3i>) {
        let mut neighbors = vec![Vec3i::new(-1, -1, -1); self.m_triangles.len()]; // TriangleSelector.cpp:424
        let mut neighbors_propagated = vec![Vec3i::new(-1, -1, -1); self.m_triangles.len()]; // TriangleSelector.cpp:425
        for facet_idx in 0..self.m_orig_size_indices {
            // TriangleSelector.cpp:426
            neighbors[facet_idx as usize] = self.m_neighbors[facet_idx as usize]; // TriangleSelector.cpp:427
            neighbors_propagated[facet_idx as usize] = neighbors[facet_idx as usize]; // TriangleSelector.cpp:428
            debug_assert!(self.verify_triangle_neighbors(&self.m_triangles[facet_idx as usize], &neighbors[facet_idx as usize])); // TriangleSelector.cpp:429
            if self.m_triangles[facet_idx as usize].is_split() {
                let n = neighbors[facet_idx as usize];
                let np = neighbors_propagated[facet_idx as usize];
                self.precompute_all_neighbors_recursive(facet_idx, &n, &np, &mut neighbors, &mut neighbors_propagated); // TriangleSelector.cpp:431
            }
        }
        (neighbors, neighbors_propagated) // TriangleSelector.cpp:433
    }
}

impl TriangleSelector {
    // ============================================================================
    // TriangleSelector.cpp:436-509 — append_touching_subtriangles / _edges / _its
    // ============================================================================

    // It appends all triangles that are touching the edge (vertexi, vertexj) of the triangle.
    // void append_touching_subtriangles(int itriangle, int vertexi, int vertexj, std::vector<int> &touching_subtriangles_out) const
    // TriangleSelector.cpp:438-459
    fn append_touching_subtriangles(&self, itriangle: i32, vertexi: i32, vertexj: i32, touching_subtriangles_out: &mut Vec<i32>) {
        if itriangle == -1 {
            return; // TriangleSelector.cpp:441
        }

        // TriangleSelector.cpp:453
        let touching = self.triangle_subtriangles(itriangle, vertexi, vertexj);

        // process_subtriangle lambda (TriangleSelector.cpp:443-451) — inlined to avoid borrow conflicts.
        let process_subtriangle = |this: &Self, subtriangle_idx: i32, partition: Partition, out: &mut Vec<i32>| {
            debug_assert!(subtriangle_idx != -1); // TriangleSelector.cpp:444
            if !this.m_triangles[subtriangle_idx as usize].is_split() {
                out.push(subtriangle_idx); // TriangleSelector.cpp:446
            } else {
                let midpoint = this.triangle_midpoint(itriangle, vertexi, vertexj); // TriangleSelector.cpp:447
                if midpoint != -1 {
                    // TriangleSelector.cpp:448
                    let (a, b) = if partition == Partition::First { (vertexi, midpoint) } else { (midpoint, vertexj) };
                    this.append_touching_subtriangles(subtriangle_idx, a, b, out);
                } else {
                    this.append_touching_subtriangles(subtriangle_idx, vertexi, vertexj, out); // TriangleSelector.cpp:450
                }
            }
        };

        if touching.0 != -1 {
            process_subtriangle(self, touching.0, Partition::First, touching_subtriangles_out); // TriangleSelector.cpp:455
        }
        if touching.1 != -1 {
            process_subtriangle(self, touching.1, Partition::Second, touching_subtriangles_out); // TriangleSelector.cpp:458
        }
    }

    // void append_touching_edges(int itriangle, int vertexi, int vertexj, std::vector<Vec2i> &touching_edges_out) const
    // TriangleSelector.cpp:463-495
    fn append_touching_edges(&self, itriangle: i32, vertexi: i32, vertexj: i32, touching_edges_out: &mut Vec<Vec2i>) {
        if itriangle == -1 {
            return; // TriangleSelector.cpp:466
        }

        let touching = self.triangle_subtriangles(itriangle, vertexi, vertexj); // TriangleSelector.cpp:489

        let process_subtriangle = |this: &Self, subtriangle_idx: i32, partition: Partition, out: &mut Vec<Vec2i>| {
            debug_assert!(subtriangle_idx != -1); // TriangleSelector.cpp:469
            if !this.m_triangles[subtriangle_idx as usize].is_split() {
                // TriangleSelector.cpp:470
                if !this.m_triangles[subtriangle_idx as usize].is_selected_by_seed_fill() {
                    // TriangleSelector.cpp:471
                    let midpoint = this.triangle_midpoint(itriangle, vertexi, vertexj); // TriangleSelector.cpp:472
                    if partition == Partition::First && midpoint != -1 {
                        out.push(Vec2i::new(vertexi, midpoint)); // TriangleSelector.cpp:474
                    } else if partition == Partition::First && midpoint == -1 {
                        out.push(Vec2i::new(vertexi, vertexj)); // TriangleSelector.cpp:476
                    } else {
                        debug_assert!(midpoint != -1 && partition == Partition::Second); // TriangleSelector.cpp:478
                        out.push(Vec2i::new(midpoint, vertexj)); // TriangleSelector.cpp:479
                    }
                }
            } else {
                let midpoint = this.triangle_midpoint(itriangle, vertexi, vertexj); // TriangleSelector.cpp:482
                if midpoint != -1 {
                    let (a, b) = if partition == Partition::First { (vertexi, midpoint) } else { (midpoint, vertexj) };
                    this.append_touching_edges(subtriangle_idx, a, b, out); // TriangleSelector.cpp:483-484
                } else {
                    this.append_touching_edges(subtriangle_idx, vertexi, vertexj, out); // TriangleSelector.cpp:486
                }
            }
        };

        if touching.0 != -1 {
            process_subtriangle(self, touching.0, Partition::First, touching_edges_out); // TriangleSelector.cpp:491
        }
        if touching.1 != -1 {
            process_subtriangle(self, touching.1, Partition::Second, touching_edges_out); // TriangleSelector.cpp:494
        }
    }

    // void append_touching_its(int itriangle, indexed_triangle_set &its) const  // TriangleSelector.cpp:497-509
    fn append_touching_its(&self, itriangle: i32, its: &mut indexed_triangle_set) {
        if itriangle == -1 {
            return; // TriangleSelector.cpp:500
        }
        let idxs = &self.m_triangles[itriangle as usize].verts_idxs; // TriangleSelector.cpp:501
        its.indices.push(StlTriangleVertexIndices::new(idxs[0], idxs[1], idxs[2])); // TriangleSelector.cpp:502
        if its.vertices.is_empty() {
            // TriangleSelector.cpp:503
            its.vertices.reserve(self.m_vertices.len()); // TriangleSelector.cpp:504
            for i in 0..self.m_vertices.len() {
                // TriangleSelector.cpp:505
                its.vertices.push(self.m_vertices[i].v); // TriangleSelector.cpp:506
            }
        }
    }

    // ============================================================================
    // TriangleSelector.cpp:511-578 — bucket_fill_select_triangles
    // ============================================================================

    // BBS: add seed_fill_angle parameter
    // void bucket_fill_select_triangles(const Vec3f& hit, int facet_start, const ClippingPlane &clp, float seed_fill_angle, bool propagate, bool force_reselection)
    // TriangleSelector.cpp:512-578
    pub fn bucket_fill_select_triangles(&mut self, hit: &Vec3f, facet_start: i32, clp: &ClippingPlane, seed_fill_angle: f32, propagate: bool, force_reselection: bool) {
        let start_facet_idx = self.select_unsplit_triangle(hit, facet_start); // TriangleSelector.cpp:514
        debug_assert!(start_facet_idx != -1); // TriangleSelector.cpp:515
        // Recompute bucket fill only if the cursor is pointing on facet unselected by bucket fill or a clipping plane is active.
        // TriangleSelector.cpp:517
        if start_facet_idx == -1 || (self.m_triangles[start_facet_idx as usize].is_selected_by_seed_fill() && !force_reselection && !clp.is_active()) {
            return; // TriangleSelector.cpp:518
        }

        debug_assert!(!self.m_triangles[start_facet_idx as usize].is_split()); // TriangleSelector.cpp:520
        let start_facet_state = self.m_triangles[start_facet_idx as usize].get_state(); // TriangleSelector.cpp:521
        self.seed_fill_unselect_all_triangles(); // TriangleSelector.cpp:522

        if !propagate {
            // TriangleSelector.cpp:524
            self.m_triangles[start_facet_idx as usize].select_by_seed_fill(); // TriangleSelector.cpp:525
            return; // TriangleSelector.cpp:526
        }

        // seed_fill_angle < 0.f to disable edge detection
        // TriangleSelector.cpp:530
        // TriangleSelector.cpp:567 — (seed_fill_angle < 0.f ? -1.f : cos(Geometry::deg2rad(seed_fill_angle))) - EPSILON
        let facet_angle_limit = if float_angle_limits() {
            (if seed_fill_angle < 0.0 { -1.0f32 } else { deg2rad_f32(seed_fill_angle).cos() }) as f64 - EPSILON
        } else {
            (if seed_fill_angle < 0.0 { -1.0 } else { deg2rad(seed_fill_angle as f64).cos() }) - EPSILON
        };

        // get_all_touching_triangles lambda (TriangleSelector.cpp:532-546) — inlined as a closure-like fn.
        let get_all_touching_triangles = |this: &Self, facet_idx: i32, neighbors: &Vec3i, neighbors_propagated: &Vec3i| -> Vec<i32> {
            debug_assert!(facet_idx != -1 && facet_idx < this.m_triangles.len() as i32); // TriangleSelector.cpp:533
            debug_assert!(this.verify_triangle_neighbors(&this.m_triangles[facet_idx as usize], neighbors)); // TriangleSelector.cpp:534
            let mut touching_triangles: Vec<i32> = Vec::new(); // TriangleSelector.cpp:535
            let vertices = Vec3i::new(this.m_triangles[facet_idx as usize].verts_idxs[0], this.m_triangles[facet_idx as usize].verts_idxs[1], this.m_triangles[facet_idx as usize].verts_idxs[2]); // TriangleSelector.cpp:536
            this.append_touching_subtriangles(neighbors[0], vertices[1], vertices[0], &mut touching_triangles); // TriangleSelector.cpp:537
            this.append_touching_subtriangles(neighbors[1], vertices[2], vertices[1], &mut touching_triangles); // TriangleSelector.cpp:538
            this.append_touching_subtriangles(neighbors[2], vertices[0], vertices[2], &mut touching_triangles); // TriangleSelector.cpp:539

            for k in 0..3usize {
                let neighbor_idx = neighbors_propagated[k]; // TriangleSelector.cpp:541
                if neighbor_idx != -1 && !this.m_triangles[neighbor_idx as usize].is_split() {
                    touching_triangles.push(neighbor_idx); // TriangleSelector.cpp:543
                }
            }
            touching_triangles // TriangleSelector.cpp:545
        };

        let (neighbors, neighbors_propagated) = self.precompute_all_neighbors(); // TriangleSelector.cpp:548
        let mut visited = vec![false; self.m_triangles.len()]; // TriangleSelector.cpp:549
        let mut facet_queue: VecDeque<i32> = VecDeque::new(); // TriangleSelector.cpp:550

        facet_queue.push_back(start_facet_idx); // TriangleSelector.cpp:552
        while let Some(current_facet) = facet_queue.pop_front() {
            // TriangleSelector.cpp:553-555
            debug_assert!(!self.m_triangles[current_facet as usize].is_split()); // TriangleSelector.cpp:556

            if !visited[current_facet as usize] {
                // TriangleSelector.cpp:558
                self.m_triangles[current_facet as usize].select_by_seed_fill(); // TriangleSelector.cpp:559

                let touching_triangles = get_all_touching_triangles(self, current_facet, &neighbors[current_facet as usize], &neighbors_propagated[current_facet as usize]); // TriangleSelector.cpp:561
                for tr_idx in touching_triangles {
                    // TriangleSelector.cpp:562
                    if tr_idx < 0 || visited[tr_idx as usize] || self.m_triangles[tr_idx as usize].get_state() != start_facet_state || self.is_facet_clipped(tr_idx, clp) {
                        continue; // TriangleSelector.cpp:563-564
                    }

                    let n1 = self.m_face_normals[self.m_triangles[tr_idx as usize].source_triangle as usize]; // TriangleSelector.cpp:566
                    let n2 = self.m_face_normals[self.m_triangles[current_facet as usize].source_triangle as usize]; // TriangleSelector.cpp:567
                    if (seed_fill_angle as f64) >= -EPSILON && (dotf(&n1, &n2).clamp(0.0, 1.0) as f64) < facet_angle_limit {
                        continue; // TriangleSelector.cpp:568-569
                    }

                    debug_assert!(!self.m_triangles[tr_idx as usize].is_split()); // TriangleSelector.cpp:571
                    facet_queue.push_back(tr_idx); // TriangleSelector.cpp:572
                }
            }

            visited[current_facet as usize] = true; // TriangleSelector.cpp:576
        }
    }

    // ============================================================================
    // TriangleSelector.cpp:580-613 — select_triangle
    // ============================================================================

    // bool select_triangle(int facet_idx, EnforcerBlockerType type, bool triangle_splitting)
    // TriangleSelector.cpp:585-613
    fn select_triangle(&mut self, facet_idx: i32, ty: EnforcerBlockerType, triangle_splitting: bool) -> bool {
        debug_assert!(facet_idx < self.m_triangles.len() as i32); // TriangleSelector.cpp:587

        if !self.m_triangles[facet_idx as usize].valid() {
            return false; // TriangleSelector.cpp:590
        }

        let neighbors = self.m_neighbors[facet_idx as usize]; // TriangleSelector.cpp:592
        debug_assert!(self.verify_triangle_neighbors(&self.m_triangles[facet_idx as usize], &neighbors)); // TriangleSelector.cpp:593

        if !self.select_triangle_recursive(facet_idx, &neighbors, ty, triangle_splitting) {
            return false; // TriangleSelector.cpp:595-596
        }

        // In case that all children are leafs and have the same state now, they may be removed.
        self.remove_useless_children(facet_idx); // TriangleSelector.cpp:600

        // Do garbage collection maybe?
        if 2 * self.m_invalid_triangles > self.m_triangles.len() as i32 {
            // TriangleSelector.cpp:609
            self.garbage_collect(); // TriangleSelector.cpp:610
        }

        true // TriangleSelector.cpp:612
    }
}

// ============================================================================
// TriangleSelector.cpp:652-770 — neighbor_child / triangle_subtriangles / triangle_midpoint
//
// Rust has no overloading. Each of these has a `const Triangle &tr` overload and an
// `int itriangle` overload in the C++. The `int` overload is the selector's method of the C++
// name; the `const Triangle &` overload is a method of that name on the triangle, handed the
// selector it belongs to (the static `triangle_subtriangles(const Triangle &, ...)` needs none).
// ============================================================================

/// An `assert` of TriangleSelector.cpp that trees built by the pin's own
/// set_facets_with_subdivision() can break (see `verify_triangle_neighbors`): off in the engine's
/// NDEBUG build, and in a debug build checked only with the diagnostic switch `TS_VERIFY` set.
macro_rules! verify_assert {
    ($cond:expr) => {
        if cfg!(debug_assertions) && crate::probe_enabled("TS_VERIFY") {
            assert!($cond);
        }
    };
}

impl Triangle {
    /// TriangleSelector.cpp:652-689 `int neighbor_child(const Triangle &tr, int vertexi, int vertexj, Partition partition) const`:
    /// child of the triangle at a CCW oriented side (vertexi, vertexj), either first or 2nd part.
    /// If the side sharing (vertexi, vertexj) is not split, return -1.
    pub fn neighbor_child(&self, selector: &TriangleSelector, vertexi: i32, vertexj: i32, partition: Partition) -> i32 {
        let tr = self;
        if tr.number_of_split_sides() == 0 {
            // If this triangle is not split, then there is no upper / lower subtriangle sharing the edge.
            return -1; // TriangleSelector.cpp:658
        }

        // Find the triangle edge.
        let edge = if tr.verts_idxs[0] == vertexi { 0 } else if tr.verts_idxs[1] == vertexi { 1 } else { 2 }; // TriangleSelector.cpp:661
        verify_assert!(tr.verts_idxs[edge] == vertexi); // TriangleSelector.cpp:662
        verify_assert!(tr.verts_idxs[next_idx_modulo(edge, 3)] == vertexj); // TriangleSelector.cpp:663

        let child_idx: usize;
        if tr.number_of_split_sides() == 1 {
            // TriangleSelector.cpp:666
            if edge != next_idx_modulo(tr.special_side() as usize, 3) {
                // A child may or may not be split at this side.
                // TriangleSelector.cpp:669
                let c = tr.children[if edge == tr.special_side() as usize { 0 } else { 1 }];
                return selector.m_triangles[c as usize].neighbor_child(selector, vertexi, vertexj, partition);
            }
            child_idx = if partition == Partition::First { 0 } else { 1 }; // TriangleSelector.cpp:670
        } else if tr.number_of_split_sides() == 2 {
            // TriangleSelector.cpp:671
            if edge == next_idx_modulo(tr.special_side() as usize, 3) {
                // A child may or may not be split at this side.
                // TriangleSelector.cpp:674
                let c = tr.children[2];
                return selector.m_triangles[c as usize].neighbor_child(selector, vertexi, vertexj, partition);
            }
            // TriangleSelector.cpp:675-677
            child_idx = if edge == tr.special_side() as usize {
                if partition == Partition::First { 0 } else { 1 }
            } else if partition == Partition::First {
                2
            } else {
                0
            };
        } else {
            debug_assert!(tr.number_of_split_sides() == 3); // TriangleSelector.cpp:679
            debug_assert!(tr.special_side() == 0); // TriangleSelector.cpp:680
            child_idx = match edge {
                0 => {
                    if partition == Partition::First { 0 } else { 1 } // TriangleSelector.cpp:682
                }
                1 => {
                    if partition == Partition::First { 1 } else { 2 } // TriangleSelector.cpp:683
                }
                _ => {
                    debug_assert!(edge == 2); // TriangleSelector.cpp:684
                    if partition == Partition::First { 2 } else { 0 } // TriangleSelector.cpp:685
                }
            };
        }
        tr.children[child_idx] // TriangleSelector.cpp:688
    }

    /// TriangleSelector.cpp:703-730 `static std::pair<int, int> triangle_subtriangles(const Triangle &tr, int vertexi, int vertexj)`
    pub fn triangle_subtriangles(&self, vertexi: i32, vertexj: i32) -> (i32, i32) {
        let tr = self;
        if tr.number_of_split_sides() == 0 {
            // If this triangle is not split, then there is no subtriangles touching the edge.
            return (-1, -1); // TriangleSelector.cpp:707
        }

        // Find the triangle edge.
        let edge = if tr.verts_idxs[0] == vertexi { 0 } else if tr.verts_idxs[1] == vertexi { 1 } else { 2 }; // TriangleSelector.cpp:710
        verify_assert!(tr.verts_idxs[edge] == vertexi); // TriangleSelector.cpp:711
        verify_assert!(tr.verts_idxs[next_idx_modulo(edge, 3)] == vertexj); // TriangleSelector.cpp:712

        if tr.number_of_split_sides() == 1 {
            // TriangleSelector.cpp:714-716
            if edge == next_idx_modulo(tr.special_side() as usize, 3) {
                (tr.children[0], tr.children[1])
            } else {
                (tr.children[if edge == tr.special_side() as usize { 0 } else { 1 }], -1)
            }
        } else if tr.number_of_split_sides() == 2 {
            // TriangleSelector.cpp:717-720
            if edge == next_idx_modulo(tr.special_side() as usize, 3) {
                (tr.children[2], -1)
            } else if edge == tr.special_side() as usize {
                (tr.children[0], tr.children[1])
            } else {
                (tr.children[2], tr.children[0])
            }
        } else {
            debug_assert!(tr.number_of_split_sides() == 3); // TriangleSelector.cpp:722
            debug_assert!(tr.special_side() == 0); // TriangleSelector.cpp:723
            // TriangleSelector.cpp:724-726
            if edge == 0 {
                (tr.children[0], tr.children[1])
            } else if edge == 1 {
                (tr.children[1], tr.children[2])
            } else {
                (tr.children[2], tr.children[0])
            }
        }
        // TriangleSelector.cpp:729 `return std::make_pair(-1, -1);` is unreachable.
    }

    /// TriangleSelector.cpp:732-763 `int triangle_midpoint(const Triangle &tr, int vertexi, int vertexj) const`:
    /// existing midpoint of the CCW oriented side (vertexi, vertexj), -1 if the side is not split.
    pub fn triangle_midpoint(&self, selector: &TriangleSelector, vertexi: i32, vertexj: i32) -> i32 {
        let tr = self;
        if tr.number_of_split_sides() == 0 {
            // If this triangle is not split, then there is no upper / lower subtriangle sharing the edge.
            return -1; // TriangleSelector.cpp:738
        }

        // Find the triangle edge.
        let edge = if tr.verts_idxs[0] == vertexi { 0 } else if tr.verts_idxs[1] == vertexi { 1 } else { 2 }; // TriangleSelector.cpp:741
        verify_assert!(tr.verts_idxs[edge] == vertexi); // TriangleSelector.cpp:742
        verify_assert!(tr.verts_idxs[next_idx_modulo(edge, 3)] == vertexj); // TriangleSelector.cpp:743

        let m_triangles = &selector.m_triangles;
        if tr.number_of_split_sides() == 1 {
            // TriangleSelector.cpp:745-748
            if edge == next_idx_modulo(tr.special_side() as usize, 3) {
                m_triangles[tr.children[0] as usize].verts_idxs[2]
            } else {
                let c = tr.children[if edge == tr.special_side() as usize { 0 } else { 1 }];
                m_triangles[c as usize].triangle_midpoint(selector, vertexi, vertexj)
            }
        } else if tr.number_of_split_sides() == 2 {
            // TriangleSelector.cpp:749-754
            if edge == next_idx_modulo(tr.special_side() as usize, 3) {
                m_triangles[tr.children[2] as usize].triangle_midpoint(selector, vertexi, vertexj)
            } else if edge == tr.special_side() as usize {
                m_triangles[tr.children[0] as usize].verts_idxs[1]
            } else {
                m_triangles[tr.children[1] as usize].verts_idxs[2]
            }
        } else {
            debug_assert!(tr.number_of_split_sides() == 3); // TriangleSelector.cpp:756
            debug_assert!(tr.special_side() == 0); // TriangleSelector.cpp:757
            // TriangleSelector.cpp:758-761
            if edge == 0 {
                m_triangles[tr.children[0] as usize].verts_idxs[1]
            } else if edge == 1 {
                m_triangles[tr.children[1] as usize].verts_idxs[2]
            } else {
                m_triangles[tr.children[2] as usize].verts_idxs[2]
            }
        }
    }
}

impl TriangleSelector {
    /// TriangleSelector.cpp:691-696 `int neighbor_child(int itriangle, int vertexi, int vertexj, Partition partition) const`:
    /// child of itriangle at a CCW oriented side (vertexi, vertexj), either first or 2nd part.
    /// If itriangle == -1 or if the side sharing (vertexi, vertexj) is not split, return -1.
    pub fn neighbor_child(&self, itriangle: i32, vertexi: i32, vertexj: i32, partition: Partition) -> i32 {
        if itriangle == -1 {
            -1
        } else {
            self.m_triangles[itriangle as usize].neighbor_child(self, vertexi, vertexj, partition)
        }
    }

    /// TriangleSelector.cpp:698-701 `std::pair<int, int> triangle_subtriangles(int itriangle, int vertexi, int vertexj) const`
    pub fn triangle_subtriangles(&self, itriangle: i32, vertexi: i32, vertexj: i32) -> (i32, i32) {
        if itriangle == -1 {
            (-1, -1)
        } else {
            self.m_triangles[itriangle as usize].triangle_subtriangles(vertexi, vertexj)
        }
    }

    /// TriangleSelector.cpp:765-770 `int triangle_midpoint(int itriangle, int vertexi, int vertexj) const`:
    /// existing midpoint of CCW oriented side (vertexi, vertexj).
    /// If itriangle == -1 or if the side sharing (vertexi, vertexj) is not split, return -1.
    pub fn triangle_midpoint(&self, itriangle: i32, vertexi: i32, vertexj: i32) -> i32 {
        if itriangle == -1 {
            -1
        } else {
            self.m_triangles[itriangle as usize].triangle_midpoint(self, vertexi, vertexj)
        }
    }

    // int triangle_midpoint_or_allocate(int itriangle, int vertexi, int vertexj)  // TriangleSelector.cpp:772-807
    // (#ifdef EXPENSIVE_DEBUG_CHECKS at 777-782 is off in the engine's build.)
    fn triangle_midpoint_or_allocate(&mut self, itriangle: i32, vertexi: i32, vertexj: i32) -> i32 {
        let mut midpoint = self.triangle_midpoint(itriangle, vertexi, vertexj); // TriangleSelector.cpp:737
        if midpoint == -1 {
            let c = 0.5 * (self.m_vertices[vertexi as usize].v + self.m_vertices[vertexj as usize].v); // TriangleSelector.cpp:739
            // Allocate a new vertex, possibly reusing the free list.
            if self.m_free_vertices_head == -1 {
                // Allocate a new vertex.
                midpoint = self.m_vertices.len() as i32; // TriangleSelector.cpp:749
                self.m_vertices.push(Vertex::new(c)); // TriangleSelector.cpp:750
            } else {
                // Reuse a vertex from the free list.
                debug_assert!(self.m_free_vertices_head >= -1 && self.m_free_vertices_head < self.m_vertices.len() as i32); // TriangleSelector.cpp:753
                midpoint = self.m_free_vertices_head; // TriangleSelector.cpp:754
                // memcpy(&m_free_vertices_head, &m_vertices[midpoint].v[0], sizeof(int))  // TriangleSelector.cpp:755
                self.m_free_vertices_head = read_i32_from_f32(self.m_vertices[midpoint as usize].v[0]);
                debug_assert!(self.m_free_vertices_head >= -1 && self.m_free_vertices_head < self.m_vertices.len() as i32); // TriangleSelector.cpp:756
                self.m_vertices[midpoint as usize].v = c; // TriangleSelector.cpp:757
            }
            debug_assert!(self.m_vertices[midpoint as usize].ref_cnt == 0); // TriangleSelector.cpp:759
        } else {
            // TriangleSelector.cpp:798-803 (#ifndef NDEBUG)
            #[cfg(debug_assertions)]
            {
                let c1 = 0.5 * (self.m_vertices[vertexi as usize].v + self.m_vertices[vertexj as usize].v);
                let c2 = self.m_vertices[midpoint as usize].v;
                let d = (c2 - c1).norm();
                verify_assert!((d as f64).abs() < EPSILON);
            }
            debug_assert!(self.m_vertices[midpoint as usize].ref_cnt > 0); // TriangleSelector.cpp:804
        }
        midpoint // TriangleSelector.cpp:806
    }
}

impl TriangleSelector {
    // ============================================================================
    // TriangleSelector.cpp:772-859 — child_neighbors
    // ============================================================================

    // Vec3i child_neighbors(const Triangle &tr, const Vec3i &neighbors, int child_idx) const  // TriangleSelector.cpp:776-859
    fn child_neighbors(&self, tr: &Triangle, neighbors: &Vec3i, child_idx: i32) -> Vec3i {
        debug_assert!(self.verify_triangle_neighbors(tr, neighbors)); // TriangleSelector.cpp:778

        debug_assert!(child_idx >= 0 && child_idx <= tr.number_of_split_sides()); // TriangleSelector.cpp:780
        let i = tr.special_side(); // TriangleSelector.cpp:781
        let j = next_idx_modulo(i as usize, 3) as i32; // TriangleSelector.cpp:782
        let k = next_idx_modulo(j as usize, 3) as i32; // TriangleSelector.cpp:783

        let mut out = Vec3i::new(0, 0, 0); // TriangleSelector.cpp:785
        match tr.number_of_split_sides() {
            1 => match child_idx {
                0 => {
                    out[0] = neighbors[i as usize]; // TriangleSelector.cpp:790
                    out[1] = self.neighbor_child(neighbors[j as usize], tr.verts_idxs[k as usize], tr.verts_idxs[j as usize], Partition::Second); // TriangleSelector.cpp:791
                    out[2] = tr.children[1]; // TriangleSelector.cpp:792
                }
                _ => {
                    debug_assert!(child_idx == 1); // TriangleSelector.cpp:795
                    out[0] = self.neighbor_child(neighbors[j as usize], tr.verts_idxs[k as usize], tr.verts_idxs[j as usize], Partition::First); // TriangleSelector.cpp:796
                    out[1] = neighbors[k as usize]; // TriangleSelector.cpp:797
                    out[2] = tr.children[0]; // TriangleSelector.cpp:798
                }
            },
            2 => match child_idx {
                0 => {
                    out[0] = self.neighbor_child(neighbors[i as usize], tr.verts_idxs[j as usize], tr.verts_idxs[i as usize], Partition::Second); // TriangleSelector.cpp:806
                    out[1] = tr.children[1]; // TriangleSelector.cpp:807
                    out[2] = self.neighbor_child(neighbors[k as usize], tr.verts_idxs[i as usize], tr.verts_idxs[k as usize], Partition::First); // TriangleSelector.cpp:808
                }
                1 => {
                    debug_assert!(child_idx == 1); // TriangleSelector.cpp:811
                    out[0] = self.neighbor_child(neighbors[i as usize], tr.verts_idxs[j as usize], tr.verts_idxs[i as usize], Partition::First); // TriangleSelector.cpp:812
                    out[1] = tr.children[2]; // TriangleSelector.cpp:813
                    out[2] = tr.children[0]; // TriangleSelector.cpp:814
                }
                _ => {
                    debug_assert!(child_idx == 2); // TriangleSelector.cpp:817
                    out[0] = neighbors[j as usize]; // TriangleSelector.cpp:818
                    out[1] = self.neighbor_child(neighbors[k as usize], tr.verts_idxs[i as usize], tr.verts_idxs[k as usize], Partition::Second); // TriangleSelector.cpp:819
                    out[2] = tr.children[1]; // TriangleSelector.cpp:820
                }
            },
            3 => {
                debug_assert!(tr.special_side() == 0); // TriangleSelector.cpp:826
                match child_idx {
                    0 => {
                        out[0] = self.neighbor_child(neighbors[0], tr.verts_idxs[1], tr.verts_idxs[0], Partition::Second); // TriangleSelector.cpp:829
                        out[1] = tr.children[3]; // TriangleSelector.cpp:830
                        out[2] = self.neighbor_child(neighbors[2], tr.verts_idxs[0], tr.verts_idxs[2], Partition::First); // TriangleSelector.cpp:831
                    }
                    1 => {
                        out[0] = self.neighbor_child(neighbors[0], tr.verts_idxs[1], tr.verts_idxs[0], Partition::First); // TriangleSelector.cpp:834
                        out[1] = self.neighbor_child(neighbors[1], tr.verts_idxs[2], tr.verts_idxs[1], Partition::Second); // TriangleSelector.cpp:835
                        out[2] = tr.children[3]; // TriangleSelector.cpp:836
                    }
                    2 => {
                        out[0] = self.neighbor_child(neighbors[1], tr.verts_idxs[2], tr.verts_idxs[1], Partition::First); // TriangleSelector.cpp:839
                        out[1] = self.neighbor_child(neighbors[2], tr.verts_idxs[0], tr.verts_idxs[2], Partition::Second); // TriangleSelector.cpp:840
                        out[2] = tr.children[3]; // TriangleSelector.cpp:841
                    }
                    _ => {
                        debug_assert!(child_idx == 3); // TriangleSelector.cpp:844
                        out[0] = tr.children[1]; // TriangleSelector.cpp:845
                        out[1] = tr.children[2]; // TriangleSelector.cpp:846
                        out[2] = tr.children[0]; // TriangleSelector.cpp:847
                    }
                }
            }
            _ => {
                debug_assert!(false); // TriangleSelector.cpp:853
            }
        }

        debug_assert!(self.verify_triangle_neighbors(tr, neighbors)); // TriangleSelector.cpp:856
        debug_assert!(self.verify_triangle_neighbors(&self.m_triangles[tr.children[child_idx as usize] as usize], &out)); // TriangleSelector.cpp:857
        out // TriangleSelector.cpp:858
    }

    // ============================================================================
    // TriangleSelector.cpp:861-933 — child_neighbors_propagated
    // ============================================================================

    // Vec3i child_neighbors_propagated(const Triangle &tr, const Vec3i &neighbors_propagated, int child_idx, const Vec3i &child_neighbors) const
    // TriangleSelector.cpp:863-933
    fn child_neighbors_propagated(&self, tr: &Triangle, neighbors_propagated: &Vec3i, child_idx: i32, child_neighbors: &Vec3i) -> Vec3i {
        let i = tr.special_side(); // TriangleSelector.cpp:865
        let j = next_idx_modulo(i as usize, 3) as i32; // TriangleSelector.cpp:866
        let k = next_idx_modulo(j as usize, 3) as i32; // TriangleSelector.cpp:867

        let mut out = *child_neighbors; // TriangleSelector.cpp:869
        // replace_if_not_exists lambda (TriangleSelector.cpp:870-873)
        let replace_if_not_exists = |out: &mut Vec3i, index_to_replace: i32, neighbor_idx: i32| {
            if out[index_to_replace as usize] == -1 {
                out[index_to_replace as usize] = neighbors_propagated[neighbor_idx as usize];
            }
        };

        match tr.number_of_split_sides() {
            1 => match child_idx {
                0 => {
                    replace_if_not_exists(&mut out, 0, i); // TriangleSelector.cpp:879
                    replace_if_not_exists(&mut out, 1, j); // TriangleSelector.cpp:880
                }
                _ => {
                    debug_assert!(child_idx == 1); // TriangleSelector.cpp:883
                    replace_if_not_exists(&mut out, 0, j); // TriangleSelector.cpp:884
                    replace_if_not_exists(&mut out, 1, k); // TriangleSelector.cpp:885
                }
            },
            2 => match child_idx {
                0 => {
                    replace_if_not_exists(&mut out, 0, i); // TriangleSelector.cpp:893
                    replace_if_not_exists(&mut out, 2, k); // TriangleSelector.cpp:894
                }
                1 => {
                    debug_assert!(child_idx == 1); // TriangleSelector.cpp:897
                    replace_if_not_exists(&mut out, 0, i); // TriangleSelector.cpp:898
                }
                _ => {
                    debug_assert!(child_idx == 2); // TriangleSelector.cpp:901
                    replace_if_not_exists(&mut out, 0, j); // TriangleSelector.cpp:902
                    replace_if_not_exists(&mut out, 1, k); // TriangleSelector.cpp:903
                }
            },
            3 => {
                debug_assert!(tr.special_side() == 0); // TriangleSelector.cpp:909
                match child_idx {
                    0 => {
                        replace_if_not_exists(&mut out, 0, 0); // TriangleSelector.cpp:912
                        replace_if_not_exists(&mut out, 2, 2); // TriangleSelector.cpp:913
                    }
                    1 => {
                        replace_if_not_exists(&mut out, 0, 0); // TriangleSelector.cpp:916
                        replace_if_not_exists(&mut out, 1, 1); // TriangleSelector.cpp:917
                    }
                    2 => {
                        replace_if_not_exists(&mut out, 0, 1); // TriangleSelector.cpp:920
                        replace_if_not_exists(&mut out, 1, 2); // TriangleSelector.cpp:921
                    }
                    _ => {
                        debug_assert!(child_idx == 3); // TriangleSelector.cpp:924
                    }
                }
            }
            _ => {
                debug_assert!(false); // TriangleSelector.cpp:929
            }
        }

        out // TriangleSelector.cpp:932
    }

    // ============================================================================
    // TriangleSelector.cpp:935-986 — select_triangle_recursive
    // ============================================================================

    // bool select_triangle_recursive(int facet_idx, const Vec3i &neighbors, EnforcerBlockerType type, bool triangle_splitting)
    // TriangleSelector.cpp:935-986
    fn select_triangle_recursive(&mut self, facet_idx: i32, neighbors: &Vec3i, ty: EnforcerBlockerType, triangle_splitting: bool) -> bool {
        debug_assert!(facet_idx < self.m_triangles.len() as i32); // TriangleSelector.cpp:937

        if !self.m_triangles[facet_idx as usize].valid() {
            return false; // TriangleSelector.cpp:940-941
        }

        debug_assert!(self.verify_triangle_neighbors(&self.m_triangles[facet_idx as usize], neighbors)); // TriangleSelector.cpp:943

        let cursor = self.m_cursor.as_ref().unwrap();
        let num_of_inside_vertices = cursor.vertices_inside(&self.m_triangles[facet_idx as usize], &self.m_vertices); // TriangleSelector.cpp:945

        // TriangleSelector.cpp:947-949
        if num_of_inside_vertices == 0
            && !cursor.is_pointer_in_triangle_tr(&self.m_triangles[facet_idx as usize], &self.m_vertices)
            && !cursor.is_edge_inside_cursor(&self.m_triangles[facet_idx as usize], &self.m_vertices)
        {
            return false; // TriangleSelector.cpp:950
        }

        if num_of_inside_vertices == 3 {
            // dump any subdivision and select whole triangle
            self.undivide_triangle(facet_idx); // TriangleSelector.cpp:954
            self.m_triangles[facet_idx as usize].set_state(ty); // TriangleSelector.cpp:955
        } else {
            // the triangle is partially inside, let's recursively divide it
            // (if not already) and try selecting its children.
            // TriangleSelector.cpp:960
            if !self.m_triangles[facet_idx as usize].is_split() && self.m_triangles[facet_idx as usize].get_state() == ty {
                // This is leaf triangle that is already of correct type as a whole.
                return true; // TriangleSelector.cpp:963
            }

            if triangle_splitting {
                // split_triangle(facet_idx, neighbors, m_cursor->uniform_scaling ? nullptr : &m_cursor->trafo);  // TriangleSelector.cpp:1004
                let cursor = self.m_cursor.as_ref().unwrap();
                let trafo = if cursor.base.uniform_scaling { None } else { Some(cursor.base.trafo) };
                self.split_triangle(facet_idx, neighbors, trafo.as_ref());
            } else if !self.m_triangles[facet_idx as usize].is_split() {
                self.m_triangles[facet_idx as usize].set_state(ty); // TriangleSelector.cpp:969
            }
            // tr = &m_triangles[facet_idx]; // might have been invalidated by split_triangle(). (TriangleSelector.cpp:970)

            let num_of_children = self.m_triangles[facet_idx as usize].number_of_split_sides() + 1; // TriangleSelector.cpp:972
            if num_of_children != 1 {
                for i in 0..num_of_children {
                    debug_assert!(i < self.m_triangles[facet_idx as usize].children.len() as i32); // TriangleSelector.cpp:975
                    debug_assert!(self.m_triangles[facet_idx as usize].children[i as usize] < self.m_triangles.len() as i32); // TriangleSelector.cpp:976
                    // Recursion, deep first search over the children of this triangle.
                    let tr_clone = self.m_triangles[facet_idx as usize].clone();
                    let child = tr_clone.children[i as usize];
                    let cn = self.child_neighbors(&tr_clone, neighbors, i); // TriangleSelector.cpp:979
                    self.select_triangle_recursive(child, &cn, ty, triangle_splitting);
                    // tr = &m_triangles[facet_idx]; // might have been invalidated (TriangleSelector.cpp:980)
                }
            }
        }

        true // TriangleSelector.cpp:985
    }

    // void set_facet(int facet_idx, EnforcerBlockerType state)  // TriangleSelector.cpp:988-994
    pub fn set_facet(&mut self, facet_idx: i32, state: EnforcerBlockerType) {
        debug_assert!(facet_idx < self.m_orig_size_indices); // TriangleSelector.cpp:990
        self.undivide_triangle(facet_idx); // TriangleSelector.cpp:991
        debug_assert!(!self.m_triangles[facet_idx as usize].is_split()); // TriangleSelector.cpp:992
        self.m_triangles[facet_idx as usize].set_state(state); // TriangleSelector.cpp:993
    }

    // ============================================================================
    // TriangleSelector.cpp:1033-1303 — set_facets_with_subdivision, facet_vertices, paint_region
    // ============================================================================

    /// TriangleSelector.cpp:1033-1141 `FacetSubdivisionResult set_facets_with_subdivision(const std::vector<int> &facet_indices,
    /// const FacetSubdivisionEvaluator &evaluator, size_t node_budget, double relative_error_limit,
    /// double absolute_error_epsilon, int max_depth, const FacetSubdivisionCancelCallback &cancel = nullptr)`.
    ///
    /// Rebuild original facets by repeatedly splitting the leaf with the largest measured error.
    /// node_budget counts newly created child nodes. If cancel is set and returns true, the loop
    /// stops early and result.canceled is set (TriangleSelector.hpp:286-288).
    ///
    /// The `std::priority_queue` is a `BinaryHeap` ordered by the C++ `CandidateCompare`. The two
    /// heaps lay their elements out differently, but `CandidateCompare` is a strict total order
    /// over the queued candidates (a leaf is queued at most once, so `facet_idx` tells any two
    /// apart, and a NaN error is never queued), so both pop the same candidate every time.
    pub fn set_facets_with_subdivision(
        &mut self,
        facet_indices: &[i32],
        evaluator: &mut FacetSubdivisionEvaluator<'_>,
        mut node_budget: usize,
        relative_error_limit: f64,
        absolute_error_epsilon: f64,
        max_depth: i32,
        mut cancel: Option<&mut FacetSubdivisionCancelCallback<'_>>,
    ) -> FacetSubdivisionResult {
        // TriangleSelector.cpp:1042-1048
        #[derive(Clone, Copy)]
        struct Candidate {
            root_idx: i32,
            facet_idx: i32,
            neighbors: Vec3i,
            depth: i32,
            measurement: FacetSubdivisionMeasurement,
        }
        // TriangleSelector.cpp:1049-1058 — CandidateCompare::operator()(lhs, rhs): lhs is "less" than rhs
        fn candidate_compare(lhs: &Candidate, rhs: &Candidate) -> bool {
            if lhs.measurement.error_area != rhs.measurement.error_area {
                return lhs.measurement.error_area < rhs.measurement.error_area;
            }
            if lhs.root_idx != rhs.root_idx {
                return lhs.root_idx > rhs.root_idx;
            }
            lhs.facet_idx > rhs.facet_idx
        }
        impl PartialEq for Candidate {
            fn eq(&self, other: &Self) -> bool {
                !candidate_compare(self, other) && !candidate_compare(other, self)
            }
        }
        impl Eq for Candidate {}
        impl PartialOrd for Candidate {
            fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
                Some(self.cmp(other))
            }
        }
        impl Ord for Candidate {
            fn cmp(&self, other: &Self) -> std::cmp::Ordering {
                if candidate_compare(self, other) {
                    std::cmp::Ordering::Less
                } else if candidate_compare(other, self) {
                    std::cmp::Ordering::Greater
                } else {
                    std::cmp::Ordering::Equal
                }
            }
        }

        // TriangleSelector.cpp:1060-1067
        let triangle_vertices = |this: &TriangleSelector, facet_idx: i32| -> [Vec3f; 3] {
            let tr = &this.m_triangles[facet_idx as usize];
            [this.m_vertices[tr.verts_idxs[0] as usize].v, this.m_vertices[tr.verts_idxs[1] as usize].v, this.m_vertices[tr.verts_idxs[2] as usize].v]
        };
        // TriangleSelector.cpp:1068-1072
        let normalize_measurement = |mut measurement: FacetSubdivisionMeasurement| -> FacetSubdivisionMeasurement {
            // std::max(0.0, measurement.surface_area)
            measurement.surface_area = if 0.0 < measurement.surface_area { measurement.surface_area } else { 0.0 };
            // std::clamp(measurement.error_area, 0.0, measurement.surface_area)
            measurement.error_area = if measurement.error_area < 0.0 {
                0.0
            } else if measurement.surface_area < measurement.error_area {
                measurement.surface_area
            } else {
                measurement.error_area
            };
            measurement
        };

        let mut result = FacetSubdivisionResult::default(); // TriangleSelector.cpp:1074
        let mut candidates: std::collections::BinaryHeap<Candidate> = std::collections::BinaryHeap::new(); // TriangleSelector.cpp:1075
        for &facet_idx in facet_indices {
            // TriangleSelector.cpp:1076
            if facet_idx < 0 || facet_idx >= self.m_orig_size_indices {
                continue; // TriangleSelector.cpp:1077-1078
            }
            self.undivide_triangle(facet_idx); // TriangleSelector.cpp:1079
            let measurement = normalize_measurement(evaluator(facet_idx, &triangle_vertices(self, facet_idx))); // TriangleSelector.cpp:1080-1081
            self.m_triangles[facet_idx as usize].set_state(measurement.dominant_state); // TriangleSelector.cpp:1082
            result.surface_area += measurement.surface_area; // TriangleSelector.cpp:1083
            result.error_area += measurement.error_area; // TriangleSelector.cpp:1084
            if measurement.error_area > 0.0 {
                // TriangleSelector.cpp:1085-1086
                candidates.push(Candidate { root_idx: facet_idx, facet_idx, neighbors: self.m_neighbors[facet_idx as usize], depth: 0, measurement });
            }
        }

        // TriangleSelector.cpp:1089-1090 — std::max(absolute_error_epsilon, relative_error_limit * result.surface_area)
        let target_error = {
            let relative = relative_error_limit * result.surface_area;
            if absolute_error_epsilon < relative { relative } else { absolute_error_epsilon }
        };
        let mut iterations_since_cancel_check: usize = 0; // TriangleSelector.cpp:1091
        while !candidates.is_empty() && result.error_area > target_error {
            // TriangleSelector.cpp:1092
            // Poll the cancel callback periodically (not every iteration, to keep the
            // per-node overhead negligible). A boundary-heavy face can push millions of
            // candidates, so this bounds worst-case latency to react to cancellation.
            if let Some(cancel) = cancel.as_mut() {
                // TriangleSelector.cpp:1096
                iterations_since_cancel_check += 1;
                if iterations_since_cancel_check >= 1024 {
                    iterations_since_cancel_check = 0; // TriangleSelector.cpp:1097
                    if cancel() {
                        result.canceled = true; // TriangleSelector.cpp:1099
                        break; // TriangleSelector.cpp:1100
                    }
                }
            }
            let candidate = candidates.pop().unwrap(); // TriangleSelector.cpp:1103-1104
            if candidate.depth >= max_depth {
                // TriangleSelector.cpp:1105
                result.depth_limit_reached = true; // TriangleSelector.cpp:1106
                continue; // TriangleSelector.cpp:1107
            }
            if node_budget < 4 {
                // TriangleSelector.cpp:1109
                result.node_budget_exhausted = true; // TriangleSelector.cpp:1110
                break; // TriangleSelector.cpp:1111
            }

            debug_assert!(self.m_triangles[candidate.facet_idx as usize].valid() && !self.m_triangles[candidate.facet_idx as usize].is_split()); // TriangleSelector.cpp:1115
            debug_assert!(self.verify_triangle_neighbors(&self.m_triangles[candidate.facet_idx as usize], &candidate.neighbors)); // TriangleSelector.cpp:1116
            // TriangleSelector.cpp:1117 — std::max(0.0, result.error_area - candidate.measurement.error_area)
            let remaining = result.error_area - candidate.measurement.error_area;
            result.error_area = if 0.0 < remaining { remaining } else { 0.0 };
            node_budget -= 4; // TriangleSelector.cpp:1118
            result.nodes_created += 4; // TriangleSelector.cpp:1119
            self.m_triangles[candidate.facet_idx as usize].set_division(3, 0); // TriangleSelector.cpp:1120
            self.perform_split(candidate.facet_idx, &candidate.neighbors, candidate.measurement.dominant_state); // TriangleSelector.cpp:1121

            for child_idx in 0..4 {
                // TriangleSelector.cpp:1123
                let parent = self.m_triangles[candidate.facet_idx as usize].clone(); // TriangleSelector.cpp:1124
                let child = parent.children[child_idx as usize]; // TriangleSelector.cpp:1125
                let neighbors = self.child_neighbors(&parent, &candidate.neighbors, child_idx); // TriangleSelector.cpp:1126
                let measurement = normalize_measurement(evaluator(candidate.root_idx, &triangle_vertices(self, child))); // TriangleSelector.cpp:1127-1128
                self.m_triangles[child as usize].set_state(measurement.dominant_state); // TriangleSelector.cpp:1129
                result.error_area += measurement.error_area; // TriangleSelector.cpp:1130
                if measurement.error_area > 0.0 {
                    // TriangleSelector.cpp:1131-1132
                    candidates.push(Candidate { root_idx: candidate.root_idx, facet_idx: child, neighbors, depth: candidate.depth + 1, measurement });
                }
            }
        }

        for &facet_idx in facet_indices {
            // TriangleSelector.cpp:1136
            if facet_idx >= 0 && facet_idx < self.m_orig_size_indices {
                self.remove_useless_children(facet_idx); // TriangleSelector.cpp:1137-1138
            }
        }
        // TriangleSelector.cpp:1139 — std::clamp(result.error_area, 0.0, result.surface_area)
        result.error_area = if result.error_area < 0.0 {
            0.0
        } else if result.surface_area < result.error_area {
            result.surface_area
        } else {
            result.error_area
        };
        result // TriangleSelector.cpp:1140
    }

    /// TriangleSelector.cpp:1143-1151 `std::array<Vec3f, 3> facet_vertices(int facet_idx) const`
    pub fn facet_vertices(&self, facet_idx: i32) -> [Vec3f; 3] {
        let tr = &self.m_triangles[facet_idx as usize]; // TriangleSelector.cpp:1145
        // TriangleSelector.cpp:1146-1150
        [self.m_vertices[tr.verts_idxs[0] as usize].v, self.m_vertices[tr.verts_idxs[1] as usize].v, self.m_vertices[tr.verts_idxs[2] as usize].v]
    }

    /// TriangleSelector.cpp:1159-1168 `static bool region_paint_canceled(RegionPaintContext &ctx)`:
    /// polls cancel every RegionPaintCancelPollInterval visited nodes and records the answer in
    /// ctx.result. Returns true once the walk has to unwind (TriangleSelector.hpp:494-496).
    pub fn region_paint_canceled(ctx: &mut RegionPaintContext<'_>) -> bool {
        // TriangleSelector.cpp:1161 — if (!ctx.cancel || ++ ctx.steps_since_cancel_check < RegionPaintCancelPollInterval)
        let cancel = match ctx.cancel.as_mut() {
            None => return false,
            Some(cancel) => cancel,
        };
        ctx.steps_since_cancel_check += 1;
        if ctx.steps_since_cancel_check < REGION_PAINT_CANCEL_POLL_INTERVAL {
            return false; // TriangleSelector.cpp:1162
        }
        ctx.steps_since_cancel_check = 0; // TriangleSelector.cpp:1163
        if !cancel() {
            return false; // TriangleSelector.cpp:1164-1165
        }
        ctx.result.canceled = true; // TriangleSelector.cpp:1166
        true // TriangleSelector.cpp:1167
    }

    /// TriangleSelector.cpp:1170-1240 `RegionPaintResult paint_region(const std::vector<int> &start_facets,
    /// const RegionCoverageEvaluator &coverage, EnforcerBlockerType new_state, float edge_limit,
    /// size_t node_budget, int max_depth, const FacetSubdivisionCancelCallback &cancel = nullptr,
    /// const std::vector<char> *paintable_facets = nullptr, const Transform3f *trafo = nullptr)`.
    ///
    /// Paint one source region onto this mesh the way a real brush stroke would: fill whole
    /// triangles the region covers completely, subdivide the ones it only partially covers, and
    /// stop at the edge_limit edge-length floor, where any remaining contact paints the triangle.
    /// paintable_facets, when given, is indexed by original facet and restricts the walk. The edge
    /// limit this installs is restored on return (TriangleSelector.hpp:314-341).
    #[allow(clippy::too_many_arguments)]
    pub fn paint_region<'a>(
        &mut self,
        start_facets: &[i32],
        coverage: Option<&'a mut RegionCoverageEvaluator<'a>>,
        new_state: EnforcerBlockerType,
        edge_limit: f32,
        node_budget: usize,
        max_depth: i32,
        cancel: Option<&'a mut FacetSubdivisionCancelCallback<'a>>,
        paintable_facets: Option<&[i8]>,
        trafo: Option<&'a Transform3f>,
    ) -> RegionPaintResult {
        // TriangleSelector.cpp:1181-1182
        let coverage = match coverage {
            Some(coverage) if self.m_orig_size_indices != 0 => coverage,
            _ => return RegionPaintResult::default(),
        };

        // Restore on the way out: every caller today owns a throw-away selector, but select_patch()
        // only refreshes the limit when the cursor radius changed, so leaking ours into a long-lived
        // selector would silently alter the next brush stroke. m_old_cursor_radius_sqr is left alone
        // on purpose - it is select_patch()'s own cache and touching it would defeat that refresh.
        let saved_edge_limit_sqr = self.m_edge_limit_sqr; // TriangleSelector.cpp:1188
        // TriangleSelector.cpp:1189 — ScopeGuard restore_edge_limit: run at the end of this function.
        self.set_edge_limit(edge_limit); // TriangleSelector.cpp:1190

        // TriangleSelector.cpp:1192 — RegionPaintContext ctx { coverage, cancel, trafo, new_state, max_depth, node_budget };
        let mut ctx = RegionPaintContext { coverage, cancel, trafo, new_state, max_depth, node_budget, steps_since_cancel_check: 0, result: RegionPaintResult::default() };

        // TriangleSelector.cpp:1194-1197
        let orig_size_indices = self.m_orig_size_indices;
        let paintable = |facet: i32| -> bool { facet >= 0 && facet < orig_size_indices && (paintable_facets.is_none() || paintable_facets.unwrap()[facet as usize] != 0) };

        let result: RegionPaintResult = 'walk: {
            // Breadth-first walk over the original faces, seeded with every face the
            // region may touch. Mirrors select_patch()'s neighbor walk, minus the camera
            // visibility test: a re-projected region has no viewing direction, so nothing
            // is occluded. The walk lets a coarse seed set still reach the whole region.
            let mut visited: Vec<i8> = vec![0; self.m_orig_size_indices as usize]; // TriangleSelector.cpp:1203
            let mut facets_to_check: Vec<i32> = Vec::with_capacity(start_facets.len() + 16); // TriangleSelector.cpp:1204-1205
            for &facet in start_facets {
                // TriangleSelector.cpp:1206
                if paintable(facet) {
                    facets_to_check.push(facet); // TriangleSelector.cpp:1207-1208
                }
            }

            let mut head: usize = 0;
            while head < facets_to_check.len() {
                // TriangleSelector.cpp:1210
                let facet = facets_to_check[head]; // TriangleSelector.cpp:1211
                head += 1;
                if visited[facet as usize] != 0 {
                    continue; // TriangleSelector.cpp:1212-1213
                }
                visited[facet as usize] = 1; // TriangleSelector.cpp:1214
                if Self::region_paint_canceled(&mut ctx) {
                    break 'walk ctx.result; // TriangleSelector.cpp:1215-1216
                }
                if !self.m_triangles[facet as usize].valid() {
                    continue; // TriangleSelector.cpp:1217-1218
                }
                let neighbors = self.m_neighbors[facet as usize];
                let painted = self.paint_region_recursive(facet, &neighbors, 0, &mut ctx); // TriangleSelector.cpp:1219
                if ctx.result.canceled {
                    break 'walk ctx.result; // TriangleSelector.cpp:1220-1221
                }
                if !painted {
                    continue; // TriangleSelector.cpp:1222-1223
                }

                ctx.result.facets_touched += 1; // TriangleSelector.cpp:1225
                // Children that all ended up with the same state collapse back into the
                // parent, so a region interior costs a single node no matter how deep the
                // refinement had to go along its boundary.
                self.remove_useless_children(facet); // TriangleSelector.cpp:1229
                for k in 0..3usize {
                    // TriangleSelector.cpp:1230
                    let neighbor = self.m_neighbors[facet as usize][k];
                    if paintable(neighbor) && visited[neighbor as usize] == 0 {
                        facets_to_check.push(neighbor); // TriangleSelector.cpp:1231-1232
                    }
                }
                if ctx.result.node_budget_exhausted {
                    break; // TriangleSelector.cpp:1233-1234
                }
            }

            if 2 * self.m_invalid_triangles > self.m_triangles.len() as i32 {
                self.garbage_collect(); // TriangleSelector.cpp:1237-1238
            }
            ctx.result // TriangleSelector.cpp:1239
        };
        self.m_edge_limit_sqr = saved_edge_limit_sqr; // TriangleSelector.cpp:1189 — restore_edge_limit
        result
    }

    /// TriangleSelector.cpp:1242-1303 `bool paint_region_recursive(int facet_idx, const Vec3i &neighbors, int depth, RegionPaintContext &ctx)`
    pub fn paint_region_recursive(&mut self, facet_idx: i32, neighbors: &Vec3i, depth: i32, ctx: &mut RegionPaintContext<'_>) -> bool {
        debug_assert!(facet_idx < self.m_triangles.len() as i32); // TriangleSelector.cpp:1247
        if !self.m_triangles[facet_idx as usize].valid() {
            return false; // TriangleSelector.cpp:1248-1249
        }

        // Polled here rather than only per seed: one coarse seed can carry a whole subdivision
        // tree, and every level below costs an AABB traversal inside coverage().
        if Self::region_paint_canceled(ctx) {
            return false; // TriangleSelector.cpp:1253-1254
        }

        let region_coverage = (ctx.coverage)(&self.facet_vertices(facet_idx)); // TriangleSelector.cpp:1256
        if region_coverage == RegionCoverage::None {
            return false; // TriangleSelector.cpp:1257-1258
        }

        if region_coverage == RegionCoverage::Full {
            // Fully inside the region: discard any subdivision and paint as a whole.
            self.undivide_triangle(facet_idx); // TriangleSelector.cpp:1262
            self.m_triangles[facet_idx as usize].set_state(ctx.new_state); // TriangleSelector.cpp:1263
            return true; // TriangleSelector.cpp:1264
        }

        // Partially covered, so the region boundary crosses this triangle. Refine and
        // recurse; split_triangle() refuses to split sides already at the edge-length
        // floor, which is what terminates the recursion.
        if !self.m_triangles[facet_idx as usize].is_split() {
            // TriangleSelector.cpp:1271
            let budget_left = ctx.result.nodes_created + 4 <= ctx.node_budget; // TriangleSelector.cpp:1272
            if !budget_left {
                ctx.result.node_budget_exhausted = true; // TriangleSelector.cpp:1273-1274
            }
            if depth >= ctx.max_depth || !budget_left {
                // Conservative rule: contact wins once refinement has to stop, so a
                // feature thinner than the leaf is preserved rather than eroded.
                self.m_triangles[facet_idx as usize].set_state(ctx.new_state); // TriangleSelector.cpp:1278
                return true; // TriangleSelector.cpp:1279
            }
            self.split_triangle(facet_idx, neighbors, ctx.trafo); // TriangleSelector.cpp:1281
            // tr = &m_triangles[facet_idx]; // might have been invalidated by split_triangle()  // TriangleSelector.cpp:1282
            if !self.m_triangles[facet_idx as usize].is_split() {
                // Every side is already at the edge-length floor.
                self.m_triangles[facet_idx as usize].set_state(ctx.new_state); // TriangleSelector.cpp:1285
                return true; // TriangleSelector.cpp:1286
            }
            ctx.result.nodes_created += self.m_triangles[facet_idx as usize].number_of_split_sides() as usize + 1; // TriangleSelector.cpp:1288
        }

        let num_of_children = self.m_triangles[facet_idx as usize].number_of_split_sides() + 1; // TriangleSelector.cpp:1291
        for i in 0..num_of_children {
            // TriangleSelector.cpp:1292
            let tr = self.m_triangles[facet_idx as usize].clone();
            debug_assert!((i as usize) < tr.children.len()); // TriangleSelector.cpp:1293
            debug_assert!(tr.children[i as usize] < self.m_triangles.len() as i32); // TriangleSelector.cpp:1294
            let child_neighbors = self.child_neighbors(&tr, neighbors, i);
            self.paint_region_recursive(tr.children[i as usize], &child_neighbors, depth + 1, ctx); // TriangleSelector.cpp:1295
            // The caller discards the whole selector on cancel, so leaving this subtree half
            // refined is fine; what matters is not walking the rest of it.
            if ctx.result.canceled {
                return true; // TriangleSelector.cpp:1298-1299
            }
            // tr = &m_triangles[facet_idx]; // might have been invalidated  // TriangleSelector.cpp:1300
        }
        true // TriangleSelector.cpp:1302
    }
}

impl TriangleSelector {
    // ============================================================================
    // TriangleSelector.cpp:996-1052 — split_triangle
    // ============================================================================

    // called by select_patch()->select_triangle()...select_triangle()
    // to decide which sides of the triangle to split and to actually split it calling set_division() and perform_split().
    // void split_triangle(int facet_idx, const Vec3i &neighbors, const Transform3f *trafo = nullptr)  // TriangleSelector.cpp:1305-1361
    // trafo is applied to the vertices before measuring the edge lengths against the edge-length
    // limit; None measures in mesh-local coordinates (TriangleSelector.hpp:503-505).
    fn split_triangle(&mut self, facet_idx: i32, neighbors: &Vec3i, trafo: Option<&Transform3f>) {
        if self.m_triangles[facet_idx as usize].is_split() {
            // The triangle is divided already.
            return; // TriangleSelector.cpp:1002
        }

        debug_assert!(self.verify_triangle_neighbors(&self.m_triangles[facet_idx as usize], neighbors)); // TriangleSelector.cpp:1006

        let old_type = self.m_triangles[facet_idx as usize].get_state(); // TriangleSelector.cpp:1008

        // If we got here, we are about to actually split the triangle.
        let limit_squared = self.m_edge_limit_sqr as f64; // TriangleSelector.cpp:1011

        let facet = self.m_triangles[facet_idx as usize].verts_idxs; // TriangleSelector.cpp:1013
        let mut pts: [Vec3f; 3] = [self.m_vertices[facet[0] as usize].v, self.m_vertices[facet[1] as usize].v, self.m_vertices[facet[2] as usize].v]; // TriangleSelector.cpp:1014-1016

        // In case the object is non-uniformly scaled, transform the points to world coords.
        // TriangleSelector.cpp:1330
        if let Some(trafo) = trafo {
            for i in 0..3usize {
                // TriangleSelector.cpp:1332 — pts_transformed[i] = (*trafo) * (*pts[i]);
                pts[i] = transform_point_f32(trafo, &pts[i]);
            }
        }

        // TriangleSelector.cpp:1028-1030
        let sides: [f64; 3] = [(pts[2] - pts[1]).norm_squared() as f64, (pts[0] - pts[2]).norm_squared() as f64, (pts[1] - pts[0]).norm_squared() as f64];

        let mut sides_to_split: Vec<i32> = Vec::with_capacity(3); // TriangleSelector.cpp:1032 (small_vector<int,3>)
        let mut side_to_keep = -1; // TriangleSelector.cpp:1033
        for pt_idx in 0..3 {
            // TriangleSelector.cpp:1034
            if sides[pt_idx as usize] > limit_squared {
                sides_to_split.push(pt_idx); // TriangleSelector.cpp:1036
            } else {
                side_to_keep = pt_idx; // TriangleSelector.cpp:1038
            }
        }
        if sides_to_split.is_empty() {
            // This shall be unselected.
            self.m_triangles[facet_idx as usize].set_division(0, 0); // TriangleSelector.cpp:1042
            return; // TriangleSelector.cpp:1043
        }

        // Save how the triangle will be split. Second argument makes sense only for one
        // or two split sides, otherwise the value is ignored.
        // TriangleSelector.cpp:1048-1049
        self.m_triangles[facet_idx as usize].set_division(sides_to_split.len() as i32, if sides_to_split.len() == 2 { side_to_keep } else { sides_to_split[0] });

        self.perform_split(facet_idx, neighbors, old_type); // TriangleSelector.cpp:1051
    }

    // ============================================================================
    // TriangleSelector.cpp:1171-1208 — undivide_triangle
    // ============================================================================

    // Recursively remove all subtriangles.
    // void undivide_triangle(int facet_idx)  // TriangleSelector.cpp:1172-1208
    fn undivide_triangle(&mut self, facet_idx: i32) {
        debug_assert!(facet_idx < self.m_triangles.len() as i32); // TriangleSelector.cpp:1174

        if self.m_triangles[facet_idx as usize].is_split() {
            // TriangleSelector.cpp:1177
            let n = self.m_triangles[facet_idx as usize].number_of_split_sides();
            for i in 0..=n {
                // TriangleSelector.cpp:1178
                let child = self.m_triangles[facet_idx as usize].children[i as usize]; // TriangleSelector.cpp:1179
                debug_assert!(self.m_triangles[child as usize].valid()); // TriangleSelector.cpp:1181
                self.undivide_triangle(child); // TriangleSelector.cpp:1182
                for j in 0..3usize {
                    // TriangleSelector.cpp:1183
                    let iv = self.m_triangles[child as usize].verts_idxs[j]; // TriangleSelector.cpp:1184
                    debug_assert!(self.m_vertices[iv as usize].ref_cnt > 0); // TriangleSelector.cpp:1186
                    self.m_vertices[iv as usize].ref_cnt -= 1;
                    if self.m_vertices[iv as usize].ref_cnt == 0 {
                        // TriangleSelector.cpp:1187
                        // Release this vertex. Chain released vertices into a linked list through ref_cnt.
                        debug_assert!(self.m_free_vertices_head >= -1 && self.m_free_vertices_head < self.m_vertices.len() as i32); // TriangleSelector.cpp:1190
                        // memcpy(&m_vertices[iv].v[0], &m_free_vertices_head, sizeof(int))  // TriangleSelector.cpp:1191
                        self.m_vertices[iv as usize].v[0] = write_i32_to_f32(self.m_free_vertices_head);
                        self.m_free_vertices_head = iv; // TriangleSelector.cpp:1192
                        debug_assert!(self.m_free_vertices_head >= -1 && self.m_free_vertices_head < self.m_vertices.len() as i32); // TriangleSelector.cpp:1193
                    }
                }
                // Chain released triangles into a linked list through children[0].
                debug_assert!(self.m_triangles[child as usize].valid()); // TriangleSelector.cpp:1197
                self.m_triangles[child as usize].m_valid = false; // TriangleSelector.cpp:1198
                debug_assert!(self.m_free_triangles_head >= -1 && self.m_free_triangles_head < self.m_triangles.len() as i32); // TriangleSelector.cpp:1199
                debug_assert!(self.m_free_triangles_head == -1 || !self.m_triangles[self.m_free_triangles_head as usize].valid()); // TriangleSelector.cpp:1200
                self.m_triangles[child as usize].children[0] = self.m_free_triangles_head; // TriangleSelector.cpp:1201
                self.m_free_triangles_head = child; // TriangleSelector.cpp:1202
                debug_assert!(self.m_free_triangles_head >= -1 && self.m_free_triangles_head < self.m_triangles.len() as i32); // TriangleSelector.cpp:1203
                self.m_invalid_triangles += 1; // TriangleSelector.cpp:1204
            }
            self.m_triangles[facet_idx as usize].set_division(0, 0); // not split  // TriangleSelector.cpp:1206
        }
    }

    // ============================================================================
    // TriangleSelector.cpp:1210-1246 — remove_useless_children
    // ============================================================================

    // void remove_useless_children(int facet_idx)  // TriangleSelector.cpp:1210-1246
    fn remove_useless_children(&mut self, facet_idx: i32) {
        // Check that all children are leafs of the same type. If not, try to
        // make them (recursive call). Remove them if sucessful.
        debug_assert!(facet_idx < self.m_triangles.len() as i32 && self.m_triangles[facet_idx as usize].valid()); // TriangleSelector.cpp:1215

        if !self.m_triangles[facet_idx as usize].is_split() {
            // This is a leaf, there nothing to do.
            return; // TriangleSelector.cpp:1221
        }

        // Call this for all non-leaf children.
        let n = self.m_triangles[facet_idx as usize].number_of_split_sides();
        for child_idx in 0..=n {
            // TriangleSelector.cpp:1225
            debug_assert!(child_idx < self.m_triangles.len() as i32 && self.m_triangles[child_idx as usize].valid()); // TriangleSelector.cpp:1226
            let c = self.m_triangles[facet_idx as usize].children[child_idx as usize];
            if self.m_triangles[c as usize].is_split() {
                // TriangleSelector.cpp:1227
                self.remove_useless_children(c); // TriangleSelector.cpp:1228
            }
        }

        // Return if a child is not leaf or two children differ in type.
        let mut first_child_type = EnforcerBlockerType::NONE; // TriangleSelector.cpp:1233
        for child_idx in 0..=n {
            // TriangleSelector.cpp:1234
            let c = self.m_triangles[facet_idx as usize].children[child_idx as usize];
            if self.m_triangles[c as usize].is_split() {
                return; // TriangleSelector.cpp:1235-1236
            }
            if child_idx == 0 {
                let c0 = self.m_triangles[facet_idx as usize].children[0];
                first_child_type = self.m_triangles[c0 as usize].get_state(); // TriangleSelector.cpp:1238
            } else if self.m_triangles[c as usize].get_state() != first_child_type {
                return; // TriangleSelector.cpp:1239-1240
            }
        }

        // If we got here, the children can be removed.
        self.undivide_triangle(facet_idx); // TriangleSelector.cpp:1244
        self.m_triangles[facet_idx as usize].set_state(first_child_type); // TriangleSelector.cpp:1245
    }

    // ============================================================================
    // TriangleSelector.cpp:1248-1300 — garbage_collect
    // ============================================================================

    // void garbage_collect()  // TriangleSelector.cpp:1248-1300
    pub fn garbage_collect(&mut self) {
        // First make a map from old to new triangle indices.
        let mut new_idx = self.m_orig_size_indices; // TriangleSelector.cpp:1251
        let mut new_triangle_indices = vec![-1i32; self.m_triangles.len()]; // TriangleSelector.cpp:1252
        for i in self.m_orig_size_indices..self.m_triangles.len() as i32 {
            // TriangleSelector.cpp:1253
            if self.m_triangles[i as usize].valid() {
                new_triangle_indices[i as usize] = new_idx;
                new_idx += 1; // TriangleSelector.cpp:1255
            }
        }

        // Now we know which vertices are not referenced anymore.
        new_idx = self.m_orig_size_vertices; // TriangleSelector.cpp:1259
        let mut new_vertices_indices = vec![-1i32; self.m_vertices.len()]; // TriangleSelector.cpp:1260
        for i in self.m_orig_size_vertices..self.m_vertices.len() as i32 {
            // TriangleSelector.cpp:1261
            debug_assert!(self.m_vertices[i as usize].ref_cnt >= 0); // TriangleSelector.cpp:1262
            if self.m_vertices[i as usize].ref_cnt != 0 {
                new_vertices_indices[i as usize] = new_idx;
                new_idx += 1; // TriangleSelector.cpp:1264
            }
        }

        // We can remove all invalid triangles and vertices that are no longer referenced.
        // TriangleSelector.cpp:1268-1270 — erase(remove_if(begin+orig, end, !valid), end)
        {
            let orig = self.m_orig_size_indices as usize;
            let tail: Vec<Triangle> = self.m_triangles.split_off(orig);
            let kept: Vec<Triangle> = tail.into_iter().filter(|tr| tr.valid()).collect();
            self.m_triangles.extend(kept);
        }
        // TriangleSelector.cpp:1271-1273
        {
            let orig = self.m_orig_size_vertices as usize;
            let tail: Vec<Vertex> = self.m_vertices.split_off(orig);
            let kept: Vec<Vertex> = tail.into_iter().filter(|vert| vert.ref_cnt != 0).collect();
            self.m_vertices.extend(kept);
        }

        // Now go through all remaining triangles and update changed indices.
        for tr in &mut self.m_triangles {
            // TriangleSelector.cpp:1276
            debug_assert!(tr.valid()); // TriangleSelector.cpp:1277

            if tr.is_split() {
                // There are children. Update their indices.
                for j in 0..=tr.number_of_split_sides() {
                    // TriangleSelector.cpp:1281
                    debug_assert!(new_triangle_indices[tr.children[j as usize] as usize] != -1); // TriangleSelector.cpp:1282
                    tr.children[j as usize] = new_triangle_indices[tr.children[j as usize] as usize]; // TriangleSelector.cpp:1283
                }
            }

            // Update indices into m_vertices. The original vertices are never touched.
            for idx in &mut tr.verts_idxs {
                // TriangleSelector.cpp:1289
                if *idx >= self.m_orig_size_vertices {
                    debug_assert!(new_vertices_indices[*idx as usize] != -1); // TriangleSelector.cpp:1291
                    *idx = new_vertices_indices[*idx as usize]; // TriangleSelector.cpp:1292
                }
            }
        }

        self.m_invalid_triangles = 0; // TriangleSelector.cpp:1297
        self.m_free_triangles_head = -1; // TriangleSelector.cpp:1298
        self.m_free_vertices_head = -1; // TriangleSelector.cpp:1299
    }
}

impl TriangleSelector {
    // ============================================================================
    // TriangleSelector.cpp:1302-1360 — ctor / reset / set_edge_limit / push_triangle
    // ============================================================================

    // TriangleSelector(const TriangleMesh& mesh, float edge_limit)  // TriangleSelector.cpp:1302-1306
    pub fn new(mesh: TriangleMesh, edge_limit: f32) -> Self {
        // m_mesh{mesh}, m_neighbors(its_face_neighbors(mesh.its)), m_face_normals(its_face_normals(mesh.its)), m_edge_limit(edge_limit)
        let its = {
            // build mesh.its from the crate's TriangleMesh representation.
            let mut its = indexed_triangle_set::default();
            its.vertices.reserve(mesh.vertices().len());
            for p in mesh.vertices() {
                its.vertices.push(StlVertex::new(p.x as f32, p.y as f32, p.z as f32));
            }
            its.indices.reserve(mesh.indices().len());
            for tri in mesh.indices() {
                its.indices.push(StlTriangleVertexIndices::new(tri.indices[0] as i32, tri.indices[1] as i32, tri.indices[2] as i32));
            }
            its
        };
        let m_neighbors = its_face_neighbors(&its);
        let m_face_normals = its_face_normals(&its);
        let mut sel = TriangleSelector {
            m_vertices: Vec::new(),
            m_triangles: Vec::new(),
            m_mesh: mesh,
            m_neighbors,
            m_face_normals,
            m_edge_limit: edge_limit, // TriangleSelector.hpp:374
            m_invalid_triangles: 0,
            m_edge_limit_sqr: 1.0, // TriangleSelector.hpp:380
            m_orig_size_vertices: 0, // TriangleSelector.hpp:383
            m_orig_size_indices: 0, // TriangleSelector.hpp:384
            m_cursor: None,
            m_old_cursor_radius_sqr: 0.0, // TriangleSelector.hpp:388
            m_free_triangles_head: -1, // TriangleSelector.hpp:435
            m_free_vertices_head: -1, // TriangleSelector.hpp:436
        };
        sel.reset(); // TriangleSelector.cpp:1305
        sel
    }

    // void reset()  // TriangleSelector.cpp:1308-1325
    pub fn reset(&mut self) {
        self.m_vertices.clear(); // TriangleSelector.cpp:1310
        self.m_triangles.clear(); // TriangleSelector.cpp:1311
        self.m_invalid_triangles = 0; // TriangleSelector.cpp:1312
        self.m_free_triangles_head = -1; // TriangleSelector.cpp:1313
        self.m_free_vertices_head = -1; // TriangleSelector.cpp:1314
        let its = self.its();
        self.m_vertices.reserve(its.vertices.len()); // TriangleSelector.cpp:1315
        for vert in &its.vertices {
            // TriangleSelector.cpp:1316
            self.m_vertices.push(Vertex::new(*vert)); // TriangleSelector.cpp:1317
        }
        self.m_triangles.reserve(its.indices.len()); // TriangleSelector.cpp:1318
        for i in 0..its.indices.len() {
            // TriangleSelector.cpp:1319
            let ind = its.indices[i]; // TriangleSelector.cpp:1320
            self.push_triangle(ind[0], ind[1], ind[2], i as i32, EnforcerBlockerType(0)); // TriangleSelector.cpp:1321
        }
        self.m_orig_size_vertices = self.m_vertices.len() as i32; // TriangleSelector.cpp:1323
        self.m_orig_size_indices = self.m_triangles.len() as i32; // TriangleSelector.cpp:1324
    }

    // void set_edge_limit(float edge_limit)  // TriangleSelector.cpp:1327-1330
    pub fn set_edge_limit(&mut self, edge_limit: f32) {
        self.m_edge_limit_sqr = edge_limit.powf(2.0); // TriangleSelector.cpp:1329
    }

    // int push_triangle(int a, int b, int c, int source_triangle, const EnforcerBlockerType state)
    // TriangleSelector.cpp:1332-1360
    fn push_triangle(&mut self, a: i32, b: i32, c: i32, source_triangle: i32, state: EnforcerBlockerType) -> i32 {
        for i in [a, b, c] {
            // TriangleSelector.cpp:1334
            debug_assert!(i >= 0 && i < self.m_vertices.len() as i32); // TriangleSelector.cpp:1335
            self.m_vertices[i as usize].ref_cnt += 1; // TriangleSelector.cpp:1336
        }
        let idx;
        if self.m_free_triangles_head == -1 {
            // Allocate a new triangle.
            debug_assert!(self.m_invalid_triangles == 0); // TriangleSelector.cpp:1341
            idx = self.m_triangles.len() as i32; // TriangleSelector.cpp:1342
            self.m_triangles.push(Triangle::new(a, b, c, source_triangle, state)); // TriangleSelector.cpp:1343
        } else {
            // Reuse triangle from the free list.
            debug_assert!(self.m_free_triangles_head >= -1 && self.m_free_triangles_head < self.m_triangles.len() as i32); // TriangleSelector.cpp:1346
            debug_assert!(!self.m_triangles[self.m_free_triangles_head as usize].valid()); // TriangleSelector.cpp:1347
            debug_assert!(self.m_invalid_triangles > 0); // TriangleSelector.cpp:1348
            idx = self.m_free_triangles_head; // TriangleSelector.cpp:1349
            self.m_free_triangles_head = self.m_triangles[idx as usize].children[0]; // TriangleSelector.cpp:1350
            self.m_invalid_triangles -= 1; // TriangleSelector.cpp:1351
            debug_assert!(self.m_free_triangles_head >= -1 && self.m_free_triangles_head < self.m_triangles.len() as i32); // TriangleSelector.cpp:1352
            debug_assert!(self.m_free_triangles_head == -1 || !self.m_triangles[self.m_free_triangles_head as usize].valid()); // TriangleSelector.cpp:1353
            debug_assert!(self.m_invalid_triangles >= 0); // TriangleSelector.cpp:1354
            debug_assert!((self.m_invalid_triangles == 0) == (self.m_free_triangles_head == -1)); // TriangleSelector.cpp:1355
            self.m_triangles[idx as usize] = Triangle::new(a, b, c, source_triangle, state); // TriangleSelector.cpp:1356
        }
        debug_assert!(self.m_triangles[idx as usize].valid()); // TriangleSelector.cpp:1358
        idx // TriangleSelector.cpp:1359
    }

    // ============================================================================
    // TriangleSelector.cpp:1362-1431 — perform_split
    // ============================================================================

    // void perform_split(int facet_idx, const Vec3i &neighbors, EnforcerBlockerType old_state)
    // TriangleSelector.cpp:1366-1431
    fn perform_split(&mut self, facet_idx: i32, neighbors: &Vec3i, old_state: EnforcerBlockerType) {
        // Reserve space for the new triangles upfront, so that the reference to this triangle will not change.
        {
            // TriangleSelector.cpp:1370
            let num_triangles_new = self.m_triangles.len() + self.m_triangles[facet_idx as usize].number_of_split_sides() as usize + 1;
            if self.m_triangles.capacity() < num_triangles_new {
                // TriangleSelector.cpp:1371
                let extra = next_highest_power_of_2(num_triangles_new).saturating_sub(self.m_triangles.len());
                self.m_triangles.reserve(extra); // TriangleSelector.cpp:1372
            }
        }

        debug_assert!(self.m_triangles[facet_idx as usize].is_split()); // TriangleSelector.cpp:1376

        // indices of triangle vertices (small_vector<int,6> in NDEBUG)  // TriangleSelector.cpp:1380
        let mut verts_idxs: Vec<i32> = Vec::with_capacity(6);
        {
            // TriangleSelector.cpp:1386 — for (int j=0, idx = special_side(); j<3; ++j, idx=next_idx_modulo(idx,3))
            let special = self.m_triangles[facet_idx as usize].special_side();
            let mut idx = special;
            for _j in 0..3 {
                verts_idxs.push(self.m_triangles[facet_idx as usize].verts_idxs[idx as usize]); // TriangleSelector.cpp:1387
                idx = next_idx_modulo(idx as usize, 3) as i32;
            }
        }

        let source_triangle = self.m_triangles[facet_idx as usize].source_triangle;
        let special_side = self.m_triangles[facet_idx as usize].special_side();
        let number_of_split_sides = self.m_triangles[facet_idx as usize].number_of_split_sides();

        // get_alloc_vertex lambda (TriangleSelector.cpp:1389-1391)
        // return triangle_midpoint_or_allocate(neighbors(edge), verts_idxs[i1], verts_idxs[i2]);

        let mut ichild = 0usize; // TriangleSelector.cpp:1393
        match number_of_split_sides {
            1 => {
                // TriangleSelector.cpp:1396 — insert(begin+2, get_alloc_vertex(next_idx_modulo(special,3), 2, 1))
                let v = self.triangle_midpoint_or_allocate(neighbors[next_idx_modulo(special_side as usize, 3)], verts_idxs[2], verts_idxs[1]);
                verts_idxs.insert(2, v);
                let c0 = self.push_triangle(verts_idxs[0], verts_idxs[1], verts_idxs[2], source_triangle, old_state); // TriangleSelector.cpp:1397
                self.m_triangles[facet_idx as usize].children[ichild] = c0;
                ichild += 1;
                let c1 = self.push_triangle(verts_idxs[2], verts_idxs[3], verts_idxs[0], source_triangle, old_state); // TriangleSelector.cpp:1398
                self.m_triangles[facet_idx as usize].children[ichild] = c1;
            }
            2 => {
                // TriangleSelector.cpp:1402-1403
                let v1 = self.triangle_midpoint_or_allocate(neighbors[special_side as usize], verts_idxs[1], verts_idxs[0]);
                verts_idxs.insert(1, v1);
                let v4 = self.triangle_midpoint_or_allocate(neighbors[prev_idx_modulo(special_side as usize, 3)], verts_idxs[0], verts_idxs[3]);
                verts_idxs.insert(4, v4);
                let c0 = self.push_triangle(verts_idxs[0], verts_idxs[1], verts_idxs[4], source_triangle, old_state); // TriangleSelector.cpp:1404
                self.m_triangles[facet_idx as usize].children[ichild] = c0;
                ichild += 1;
                let c1 = self.push_triangle(verts_idxs[1], verts_idxs[2], verts_idxs[4], source_triangle, old_state); // TriangleSelector.cpp:1405
                self.m_triangles[facet_idx as usize].children[ichild] = c1;
                ichild += 1;
                let c2 = self.push_triangle(verts_idxs[2], verts_idxs[3], verts_idxs[4], source_triangle, old_state); // TriangleSelector.cpp:1406
                self.m_triangles[facet_idx as usize].children[ichild] = c2;
            }
            3 => {
                debug_assert!(special_side == 0); // TriangleSelector.cpp:1410
                let v1 = self.triangle_midpoint_or_allocate(neighbors[0], verts_idxs[1], verts_idxs[0]); // TriangleSelector.cpp:1411
                verts_idxs.insert(1, v1);
                let v3 = self.triangle_midpoint_or_allocate(neighbors[1], verts_idxs[3], verts_idxs[2]); // TriangleSelector.cpp:1412
                verts_idxs.insert(3, v3);
                let v5 = self.triangle_midpoint_or_allocate(neighbors[2], verts_idxs[0], verts_idxs[4]); // TriangleSelector.cpp:1413
                verts_idxs.insert(5, v5);
                let c0 = self.push_triangle(verts_idxs[0], verts_idxs[1], verts_idxs[5], source_triangle, old_state); // TriangleSelector.cpp:1414
                self.m_triangles[facet_idx as usize].children[ichild] = c0;
                ichild += 1;
                let c1 = self.push_triangle(verts_idxs[1], verts_idxs[2], verts_idxs[3], source_triangle, old_state); // TriangleSelector.cpp:1415
                self.m_triangles[facet_idx as usize].children[ichild] = c1;
                ichild += 1;
                let c2 = self.push_triangle(verts_idxs[3], verts_idxs[4], verts_idxs[5], source_triangle, old_state); // TriangleSelector.cpp:1416
                self.m_triangles[facet_idx as usize].children[ichild] = c2;
                ichild += 1;
                let c3 = self.push_triangle(verts_idxs[1], verts_idxs[3], verts_idxs[5], source_triangle, old_state); // TriangleSelector.cpp:1417
                self.m_triangles[facet_idx as usize].children[ichild] = c3;
            }
            _ => {} // TriangleSelector.cpp:1420-1421
        }
        let _ = ichild;

        // NDEBUG verify (TriangleSelector.cpp:1424-1430) — omitted in release.
    }
}

impl TriangleSelector {
    // ============================================================================
    // TriangleSelector.cpp:1433-1496 — has_facets / num_facets / get_facets
    // ============================================================================

    // bool has_facets(EnforcerBlockerType state) const  // TriangleSelector.cpp:1433-1439
    pub fn has_facets(&self, state: EnforcerBlockerType) -> bool {
        for tr in &self.m_triangles {
            // TriangleSelector.cpp:1435
            if tr.valid() && !tr.is_split() && tr.get_state() == state {
                return true; // TriangleSelector.cpp:1437
            }
        }
        false // TriangleSelector.cpp:1438
    }

    // int num_facets(EnforcerBlockerType state) const  // TriangleSelector.cpp:1441-1448
    pub fn num_facets(&self, state: EnforcerBlockerType) -> i32 {
        let mut cnt = 0; // TriangleSelector.cpp:1443
        for tr in &self.m_triangles {
            // TriangleSelector.cpp:1444
            if tr.valid() && !tr.is_split() && tr.get_state() == state {
                cnt += 1; // TriangleSelector.cpp:1446
            }
        }
        cnt // TriangleSelector.cpp:1447
    }

    // indexed_triangle_set get_facets(EnforcerBlockerType state) const  // TriangleSelector.cpp:1450-1469
    pub fn get_facets(&self, state: EnforcerBlockerType) -> indexed_triangle_set {
        let mut out = indexed_triangle_set::default(); // TriangleSelector.cpp:1452
        let mut vertex_map = vec![-1i32; self.m_vertices.len()]; // TriangleSelector.cpp:1453
        for tr in &self.m_triangles {
            // TriangleSelector.cpp:1454
            if tr.valid() && !tr.is_split() && tr.get_state() == state {
                let mut indices = StlTriangleVertexIndices::new(0, 0, 0); // TriangleSelector.cpp:1456
                for i in 0..3usize {
                    // TriangleSelector.cpp:1457
                    let j = tr.verts_idxs[i]; // TriangleSelector.cpp:1458
                    if vertex_map[j as usize] == -1 {
                        // TriangleSelector.cpp:1459
                        vertex_map[j as usize] = out.vertices.len() as i32; // TriangleSelector.cpp:1460
                        out.vertices.push(self.m_vertices[j as usize].v); // TriangleSelector.cpp:1461
                    }
                    indices[i] = vertex_map[j as usize]; // TriangleSelector.cpp:1463
                }
                out.indices.push(indices); // TriangleSelector.cpp:1465
            }
        }
        out // TriangleSelector.cpp:1468
    }

    // BBS — void get_facets(std::vector<indexed_triangle_set>& facets_per_type) const  // TriangleSelector.cpp:1780-1810
    // The second overload of `get_facets` (Rust has no overloading).
    //
    // Gate `EXTRUDER_MAX_PIN` (faithful): on, the pin's single pass over the triangles with
    // `ExtruderMax + 1` = 97 entries; off, the earlier per-state passes with 33 entries. Both
    // bodies give the same sets for the same number of entries.
    pub fn get_facets_per_type(&self, facets_per_type: &mut Vec<indexed_triangle_set>) {
        if crate::faithful_gate("EXTRUDER_MAX_PIN") {
            let num_types = EnforcerBlockerType::extruder_max().0 as usize + 1; // TriangleSelector.cpp:1783
            facets_per_type.clear(); // TriangleSelector.cpp:1784
            facets_per_type.resize(num_types, indexed_triangle_set::default()); // TriangleSelector.cpp:1785

            // Single pass over the triangles; a vertex map is only allocated for states that actually occur.
            let mut vertex_maps: Vec<Vec<i32>> = vec![Vec::new(); num_types]; // TriangleSelector.cpp:1788
            for tr in &self.m_triangles {
                // TriangleSelector.cpp:1789
                if !tr.valid() || tr.is_split() {
                    continue; // TriangleSelector.cpp:1790-1791
                }
                let ty = tr.get_state().0 as i32; // TriangleSelector.cpp:1792
                if ty < 0 || ty >= num_types as i32 {
                    continue; // TriangleSelector.cpp:1793-1794
                }
                let vertex_map = &mut vertex_maps[ty as usize]; // TriangleSelector.cpp:1795
                let its = &mut facets_per_type[ty as usize]; // TriangleSelector.cpp:1796
                if vertex_map.is_empty() {
                    *vertex_map = vec![-1i32; self.m_vertices.len()]; // TriangleSelector.cpp:1797-1798
                }
                let mut indices = StlTriangleVertexIndices::new(0, 0, 0); // TriangleSelector.cpp:1799
                for i in 0..3usize {
                    // TriangleSelector.cpp:1800
                    let j = tr.verts_idxs[i]; // TriangleSelector.cpp:1801
                    if vertex_map[j as usize] == -1 {
                        // TriangleSelector.cpp:1802
                        vertex_map[j as usize] = its.vertices.len() as i32; // TriangleSelector.cpp:1803
                        its.vertices.push(self.m_vertices[j as usize].v); // TriangleSelector.cpp:1804
                    }
                    indices[i] = vertex_map[j as usize]; // TriangleSelector.cpp:1806
                }
                its.indices.push(indices); // TriangleSelector.cpp:1808
            }
            return;
        }

        // EXTRUDER_MAX_PIN=0: the earlier per-state passes, up to the earlier ExtruderMax (32).
        facets_per_type.clear();
        for type_i in (EnforcerBlockerType::NONE.0 as i32)..=(EnforcerBlockerType::EXTRUDER_MAX.0 as i32) {
            facets_per_type.push(indexed_triangle_set::default());
            let its = facets_per_type.last_mut().unwrap();
            let mut vertex_map = vec![-1i32; self.m_vertices.len()];

            for tr in &self.m_triangles {
                if tr.valid() && !tr.is_split() && tr.get_state() == EnforcerBlockerType(type_i as i8) {
                    let mut indices = StlTriangleVertexIndices::new(0, 0, 0);
                    for i in 0..3usize {
                        let j = tr.verts_idxs[i];
                        if vertex_map[j as usize] == -1 {
                            vertex_map[j as usize] = its.vertices.len() as i32;
                            its.vertices.push(self.m_vertices[j as usize].v);
                        }
                        indices[i] = vertex_map[j as usize];
                    }
                    its.indices.push(indices);
                }
            }
        }
    }

    // ============================================================================
    // TriangleSelector.cpp:1498-1622 — get_facets_strict[_recursive] / get_facets_split_by_tjoints
    // ============================================================================

    // indexed_triangle_set get_facets_strict(EnforcerBlockerType state) const  // TriangleSelector.cpp:1498-1522
    pub fn get_facets_strict(&self, state: EnforcerBlockerType) -> indexed_triangle_set {
        let mut out = indexed_triangle_set::default(); // TriangleSelector.cpp:1500

        let mut num_vertices = 0usize; // TriangleSelector.cpp:1502
        for v in &self.m_vertices {
            // TriangleSelector.cpp:1503
            if v.ref_cnt > 0 {
                num_vertices += 1; // TriangleSelector.cpp:1505
            }
        }
        out.vertices.reserve(num_vertices); // TriangleSelector.cpp:1506
        let mut vertex_map = vec![-1i32; self.m_vertices.len()]; // TriangleSelector.cpp:1507
        for i in 0..self.m_vertices.len() {
            // TriangleSelector.cpp:1508
            let v = &self.m_vertices[i];
            if v.ref_cnt > 0 {
                vertex_map[i] = out.vertices.len() as i32; // TriangleSelector.cpp:1510
                out.vertices.push(v.v); // TriangleSelector.cpp:1511
            }
        }

        for itriangle in 0..self.m_orig_size_indices {
            // TriangleSelector.cpp:1514
            let tr = self.m_triangles[itriangle as usize].clone();
            let n = self.m_neighbors[itriangle as usize];
            self.get_facets_strict_recursive(&tr, &n, state, &mut out.indices); // TriangleSelector.cpp:1515
        }

        for triangle in &mut out.indices {
            // TriangleSelector.cpp:1517
            for i in 0..3usize {
                triangle[i] = vertex_map[triangle[i] as usize]; // TriangleSelector.cpp:1519
            }
        }

        out // TriangleSelector.cpp:1521
    }

    // void get_facets_strict_recursive(const Triangle &tr, const Vec3i &neighbors, EnforcerBlockerType state, std::vector<stl_triangle_vertex_indices> &out_triangles) const
    // TriangleSelector.cpp:1524-1538
    fn get_facets_strict_recursive(&self, tr: &Triangle, neighbors: &Vec3i, state: EnforcerBlockerType, out_triangles: &mut Vec<StlTriangleVertexIndices>) {
        if tr.is_split() {
            // TriangleSelector.cpp:1530
            for i in 0..=tr.number_of_split_sides() {
                // TriangleSelector.cpp:1531
                let child = self.m_triangles[tr.children[i as usize] as usize].clone();
                let cn = self.child_neighbors(tr, neighbors, i);
                self.get_facets_strict_recursive(&child, &cn, state, out_triangles); // TriangleSelector.cpp:1532-1535
            }
        } else if tr.get_state() == state {
            // TriangleSelector.cpp:1536
            self.get_facets_split_by_tjoints(&Vec3i::new(tr.verts_idxs[0], tr.verts_idxs[1], tr.verts_idxs[2]), neighbors, out_triangles); // TriangleSelector.cpp:1537
        }
    }

    // void get_facets_split_by_tjoints(const Vec3i &vertices, const Vec3i &neighbors, std::vector<stl_triangle_vertex_indices> &out_triangles) const
    // TriangleSelector.cpp:1540-1622
    fn get_facets_split_by_tjoints(&self, vertices: &Vec3i, neighbors: &Vec3i, out_triangles: &mut Vec<StlTriangleVertexIndices>) {
        // Export this triangle, but first collect the T-joint vertices along its edges.
        // TriangleSelector.cpp:1543-1546
        let midpoints = Vec3i::new(
            self.triangle_midpoint(neighbors[0], vertices[1], vertices[0]),
            self.triangle_midpoint(neighbors[1], vertices[2], vertices[1]),
            self.triangle_midpoint(neighbors[2], vertices[0], vertices[2]),
        );
        let splits = (midpoints[0] != -1) as i32 + (midpoints[1] != -1) as i32 + (midpoints[2] != -1) as i32; // TriangleSelector.cpp:1547
        match splits {
            0 => {
                // Just emit this triangle.
                out_triangles.push(StlTriangleVertexIndices::new(vertices[0], vertices[1], vertices[2])); // TriangleSelector.cpp:1551
            }
            1 => {
                // Split to two triangles
                let i = if midpoints[0] != -1 { 2 } else if midpoints[1] != -1 { 0 } else { 1 }; // TriangleSelector.cpp:1556
                let j = next_idx_modulo(i as usize, 3) as i32; // TriangleSelector.cpp:1557
                let k = next_idx_modulo(j as usize, 3) as i32; // TriangleSelector.cpp:1558
                self.get_facets_split_by_tjoints(
                    &Vec3i::new(vertices[i as usize], vertices[j as usize], midpoints[j as usize]), // TriangleSelector.cpp:1560
                    &Vec3i::new(neighbors[i as usize], self.neighbor_child(neighbors[j as usize], vertices[k as usize], vertices[j as usize], Partition::Second), -1), // TriangleSelector.cpp:1561-1563
                    out_triangles,
                );
                self.get_facets_split_by_tjoints(
                    &Vec3i::new(midpoints[j as usize], vertices[k as usize], vertices[i as usize]), // TriangleSelector.cpp:1566
                    &Vec3i::new(self.neighbor_child(neighbors[j as usize], vertices[k as usize], vertices[j as usize], Partition::First), neighbors[k as usize], -1), // TriangleSelector.cpp:1567-1569
                    out_triangles,
                );
            }
            2 => {
                // Split to three triangles.
                let i = if midpoints[0] == -1 { 2 } else if midpoints[1] == -1 { 0 } else { 1 }; // TriangleSelector.cpp:1576
                let j = next_idx_modulo(i as usize, 3) as i32; // TriangleSelector.cpp:1577
                let k = next_idx_modulo(j as usize, 3) as i32; // TriangleSelector.cpp:1578
                self.get_facets_split_by_tjoints(
                    &Vec3i::new(vertices[i as usize], midpoints[i as usize], midpoints[k as usize]), // TriangleSelector.cpp:1580
                    &Vec3i::new(self.neighbor_child(neighbors[i as usize], vertices[j as usize], vertices[i as usize], Partition::Second), -1, self.neighbor_child(neighbors[k as usize], vertices[i as usize], vertices[k as usize], Partition::First)), // TriangleSelector.cpp:1581-1583
                    out_triangles,
                );
                self.get_facets_split_by_tjoints(
                    &Vec3i::new(midpoints[i as usize], vertices[j as usize], midpoints[k as usize]), // TriangleSelector.cpp:1586
                    &Vec3i::new(self.neighbor_child(neighbors[i as usize], vertices[j as usize], vertices[i as usize], Partition::First), -1, -1), // TriangleSelector.cpp:1587-1588
                    out_triangles,
                );
                self.get_facets_split_by_tjoints(
                    &Vec3i::new(vertices[j as usize], vertices[k as usize], midpoints[k as usize]), // TriangleSelector.cpp:1591
                    &Vec3i::new(neighbors[j as usize], self.neighbor_child(neighbors[k as usize], vertices[i as usize], vertices[k as usize], Partition::Second), -1), // TriangleSelector.cpp:1592-1594
                    out_triangles,
                );
            }
            _ => {
                debug_assert!(splits == 3); // TriangleSelector.cpp:1599
                // Split to 4 triangles.
                self.get_facets_split_by_tjoints(
                    &Vec3i::new(vertices[0], midpoints[0], midpoints[2]), // TriangleSelector.cpp:1602
                    &Vec3i::new(self.neighbor_child(neighbors[0], vertices[1], vertices[0], Partition::Second), -1, self.neighbor_child(neighbors[2], vertices[0], vertices[2], Partition::First)), // TriangleSelector.cpp:1603-1605
                    out_triangles,
                );
                self.get_facets_split_by_tjoints(
                    &Vec3i::new(midpoints[0], vertices[1], midpoints[1]), // TriangleSelector.cpp:1608
                    &Vec3i::new(self.neighbor_child(neighbors[0], vertices[1], vertices[0], Partition::First), self.neighbor_child(neighbors[1], vertices[2], vertices[1], Partition::Second), -1), // TriangleSelector.cpp:1609-1611
                    out_triangles,
                );
                self.get_facets_split_by_tjoints(
                    &Vec3i::new(midpoints[1], vertices[2], midpoints[2]), // TriangleSelector.cpp:1614
                    &Vec3i::new(self.neighbor_child(neighbors[1], vertices[2], vertices[1], Partition::First), self.neighbor_child(neighbors[2], vertices[0], vertices[2], Partition::Second), -1), // TriangleSelector.cpp:1615-1617
                    out_triangles,
                );
                out_triangles.push(midpoints); // TriangleSelector.cpp:1619
            }
        }
    }
}

impl TriangleSelector {
    // ============================================================================
    // TriangleSelector.cpp:1624-1720 — get_seed_fill_contour / mesh / recursive
    // ============================================================================

    // std::vector<Vec2i> get_seed_fill_contour() const  // TriangleSelector.cpp:1624-1633
    pub fn get_seed_fill_contour(&self) -> Vec<Vec2i> {
        let mut edges_out: Vec<Vec2i> = Vec::new(); // TriangleSelector.cpp:1625
        for facet_idx in 0..self.m_orig_size_indices {
            // TriangleSelector.cpp:1626
            let neighbors = self.m_neighbors[facet_idx as usize]; // TriangleSelector.cpp:1627
            debug_assert!(self.verify_triangle_neighbors(&self.m_triangles[facet_idx as usize], &neighbors)); // TriangleSelector.cpp:1628
            self.get_seed_fill_contour_recursive(facet_idx, &neighbors, &neighbors, &mut edges_out); // TriangleSelector.cpp:1629
        }
        edges_out // TriangleSelector.cpp:1632
    }

    // indexed_triangle_set get_seed_fill_mesh(int &state) const  // TriangleSelector.cpp:1635-1644
    pub fn get_seed_fill_mesh(&self, state: &mut i32) -> indexed_triangle_set {
        let mut its = indexed_triangle_set::default(); // TriangleSelector.cpp:1637
        let mut face_idx_set: BTreeSet<i32> = BTreeSet::new(); // TriangleSelector.cpp:1638 (std::set)
        for facet_idx in 0..self.m_orig_size_indices {
            // TriangleSelector.cpp:1639
            let neighbors = self.m_neighbors[facet_idx as usize]; // TriangleSelector.cpp:1640
            self.get_seed_fill_its_recursive(facet_idx, &neighbors, &neighbors, &mut face_idx_set, &mut its, state); // TriangleSelector.cpp:1641
        }
        its // TriangleSelector.cpp:1643
    }

    // void get_seed_fill_its_recursive(int facet_idx, const Vec3i &neighbors, const Vec3i &neighbors_propagated, std::set<int> &idx_set, indexed_triangle_set &its, int &state) const
    // TriangleSelector.cpp:1646-1686
    fn get_seed_fill_its_recursive(&self, facet_idx: i32, neighbors: &Vec3i, neighbors_propagated: &Vec3i, idx_set: &mut BTreeSet<i32>, its: &mut indexed_triangle_set, state: &mut i32) {
        debug_assert!(facet_idx != -1 && facet_idx < self.m_triangles.len() as i32); // TriangleSelector.cpp:1649
        debug_assert!(self.verify_triangle_neighbors(&self.m_triangles[facet_idx as usize], neighbors)); // TriangleSelector.cpp:1650
        let tr = &self.m_triangles[facet_idx as usize]; // TriangleSelector.cpp:1651
        if !tr.valid() {
            return; // TriangleSelector.cpp:1653
        }

        if tr.is_split() {
            // TriangleSelector.cpp:1655
            let num_of_children = tr.number_of_split_sides() + 1; // TriangleSelector.cpp:1656
            if num_of_children != 1 {
                for i in 0..num_of_children {
                    debug_assert!(i < tr.children.len() as i32); // TriangleSelector.cpp:1659
                    debug_assert!(tr.children[i as usize] < self.m_triangles.len() as i32); // TriangleSelector.cpp:1660
                    let tr_clone = tr.clone();
                    let child_neighbors = self.child_neighbors(&tr_clone, neighbors, i); // TriangleSelector.cpp:1663
                    let cnp = self.child_neighbors_propagated(&tr_clone, neighbors_propagated, i, &child_neighbors);
                    self.get_seed_fill_its_recursive(tr_clone.children[i as usize], &child_neighbors, &cnp, idx_set, its, state); // TriangleSelector.cpp:1664-1665
                }
            }
        } else if tr.is_selected_by_seed_fill() {
            // TriangleSelector.cpp:1668
            let select_state = self.m_triangles[facet_idx as usize].get_state(); // TriangleSelector.cpp:1669
            if *state < 0 {
                // TriangleSelector.cpp:1670
                *state = select_state.0 as i32; // TriangleSelector.cpp:1671
            }
            if !idx_set.contains(&facet_idx) {
                // TriangleSelector.cpp:1673
                idx_set.insert(facet_idx); // TriangleSelector.cpp:1674
                self.append_touching_its(facet_idx, its); // TriangleSelector.cpp:1675
                for i in 0..3usize {
                    // TriangleSelector.cpp:1676
                    if neighbors[i] >= 0 && (neighbors[i] as usize) < self.m_triangles.len() && select_state == self.m_triangles[neighbors[i] as usize].get_state() {
                        // TriangleSelector.cpp:1677
                        if !idx_set.contains(&neighbors[i]) {
                            // TriangleSelector.cpp:1678
                            idx_set.insert(neighbors[i]); // TriangleSelector.cpp:1679
                            self.append_touching_its(neighbors[i], its); // TriangleSelector.cpp:1680
                        }
                    }
                }
            }
        }
    }

    // void get_seed_fill_contour_recursive(int facet_idx, const Vec3i &neighbors, const Vec3i &neighbors_propagated, std::vector<Vec2i> &edges_out) const
    // TriangleSelector.cpp:1688-1720
    fn get_seed_fill_contour_recursive(&self, facet_idx: i32, neighbors: &Vec3i, neighbors_propagated: &Vec3i, edges_out: &mut Vec<Vec2i>) {
        debug_assert!(facet_idx != -1 && facet_idx < self.m_triangles.len() as i32); // TriangleSelector.cpp:1690
        debug_assert!(self.verify_triangle_neighbors(&self.m_triangles[facet_idx as usize], neighbors)); // TriangleSelector.cpp:1691
        let tr = &self.m_triangles[facet_idx as usize]; // TriangleSelector.cpp:1692
        if !tr.valid() {
            return; // TriangleSelector.cpp:1694
        }

        if tr.is_split() {
            // TriangleSelector.cpp:1696
            let num_of_children = tr.number_of_split_sides() + 1; // TriangleSelector.cpp:1697
            if num_of_children != 1 {
                for i in 0..num_of_children {
                    debug_assert!(i < tr.children.len() as i32); // TriangleSelector.cpp:1700
                    debug_assert!(tr.children[i as usize] < self.m_triangles.len() as i32); // TriangleSelector.cpp:1701
                    let tr_clone = tr.clone();
                    let child_neighbors = self.child_neighbors(&tr_clone, neighbors, i); // TriangleSelector.cpp:1704
                    let cnp = self.child_neighbors_propagated(&tr_clone, neighbors_propagated, i, &child_neighbors);
                    self.get_seed_fill_contour_recursive(tr_clone.children[i as usize], &child_neighbors, &cnp, edges_out); // TriangleSelector.cpp:1705-1706
                }
            }
        } else if tr.is_selected_by_seed_fill() {
            // TriangleSelector.cpp:1709
            let vertices = Vec3i::new(self.m_triangles[facet_idx as usize].verts_idxs[0], self.m_triangles[facet_idx as usize].verts_idxs[1], self.m_triangles[facet_idx as usize].verts_idxs[2]); // TriangleSelector.cpp:1710
            self.append_touching_edges(neighbors[0], vertices[1], vertices[0], edges_out); // TriangleSelector.cpp:1711
            self.append_touching_edges(neighbors[1], vertices[2], vertices[1], edges_out); // TriangleSelector.cpp:1712
            self.append_touching_edges(neighbors[2], vertices[0], vertices[2], edges_out); // TriangleSelector.cpp:1713

            // It appends the edges that are touching the triangle only by part of the edge.
            for idx in 0..3usize {
                // TriangleSelector.cpp:1716
                let neighbor_tr_idx = neighbors_propagated[idx];
                if neighbor_tr_idx != -1 && !self.m_triangles[neighbor_tr_idx as usize].is_split() && !self.m_triangles[neighbor_tr_idx as usize].is_selected_by_seed_fill() {
                    // TriangleSelector.cpp:1717
                    edges_out.push(Vec2i::new(vertices[idx], vertices[next_idx_modulo(idx, 3)])); // TriangleSelector.cpp:1718
                }
            }
        }
    }
}

/// Serialized form: `std::pair<std::vector<std::pair<int,int>>, std::vector<bool>>`
/// (TriangleSelector.hpp:275). `.0` = (triangle index, first bit), `.1` = bit stream.
pub type SerializedData = (Vec<(i32, i32)>, Vec<bool>);

impl TriangleSelector {
    /// Distinct non-NONE painted states over all valid triangles (leaves and
    /// split parents alike), ascending.
    ///
    /// For painted-MMU annotations the state value IS the filament/extruder
    /// slot (EXTRUDER1=1, EXTRUDER2=2, … — Model.hpp:713 `EnforcerBlockerType`,
    /// "BBS: painting" comment), so this yields the set of painted extruders —
    /// the input `generate_print_object_regions` (PrintApply.cpp:1062) walks
    /// as `painting_extruders`.
    ///
    /// Sized by `ExtruderMax + 1`: gate `EXTRUDER_MAX_PIN` (faithful) gives the pin's 97 entries,
    /// `EXTRUDER_MAX_PIN=0` the earlier 33.
    pub fn used_states(&self) -> Vec<EnforcerBlockerType> {
        let mut seen = vec![false; EnforcerBlockerType::extruder_max().0 as usize + 1];
        for tr in self.m_triangles.iter() {
            if !tr.valid() {
                continue;
            }
            let s = tr.get_state();
            if s.0 > 0 && (s.0 as usize) < seen.len() {
                seen[s.0 as usize] = true;
            }
        }
        (1..seen.len())
            .filter(|&i| seen[i])
            .map(|i| EnforcerBlockerType(i as i8))
            .collect()
    }

    // ============================================================================
    // TriangleSelector.cpp:1722-1799 — serialize
    // ============================================================================

    // std::pair<std::vector<std::pair<int, int>>, std::vector<bool>> serialize() const  // TriangleSelector.cpp:1722-1799
    pub fn serialize(&self) -> SerializedData {
        // struct Serializer { ... } (TriangleSelector.cpp:1737-1784) — modeled as a recursive method.
        let mut data: SerializedData = (Vec::new(), Vec::new());

        data.0.reserve(self.m_orig_size_indices as usize); // TriangleSelector.cpp:1786
        for i in 0..self.m_orig_size_indices {
            // TriangleSelector.cpp:1787
            let tr = &self.m_triangles[i as usize];
            if tr.is_split() || tr.get_state() != EnforcerBlockerType::NONE {
                // Store index of the first bit assigned to ith triangle.
                data.0.push((i, data.1.len() as i32)); // TriangleSelector.cpp:1790
                // out the triangle bits.
                self.serialize_recursive(i, &mut data); // TriangleSelector.cpp:1792
            }
        }

        // May be stored onto Undo / Redo stack, thus conserve memory.
        data.0.shrink_to_fit(); // TriangleSelector.cpp:1796
        data.1.shrink_to_fit(); // TriangleSelector.cpp:1797
        data // TriangleSelector.cpp:1798
    }

    // Serializer::serialize(int facet_idx)  // TriangleSelector.cpp:1741-1783
    fn serialize_recursive(&self, facet_idx: i32, data: &mut SerializedData) {
        let tr = &self.m_triangles[facet_idx as usize]; // TriangleSelector.cpp:1742

        // Always save number of split sides. It is zero for unsplit triangles.
        let split_sides = tr.number_of_split_sides(); // TriangleSelector.cpp:1745
        debug_assert!(split_sides >= 0 && split_sides <= 3); // TriangleSelector.cpp:1746

        data.1.push((split_sides & 0b01) != 0); // TriangleSelector.cpp:1748
        data.1.push((split_sides & 0b10) != 0); // TriangleSelector.cpp:1749

        if split_sides != 0 {
            // TriangleSelector.cpp:1751
            // If this triangle is split, save which side is split (or kept).
            debug_assert!(tr.is_split() && split_sides > 0); // TriangleSelector.cpp:1755
            debug_assert!(tr.special_side() >= 0 && tr.special_side() <= 3); // TriangleSelector.cpp:1756
            data.1.push((tr.special_side() & 0b01) != 0); // TriangleSelector.cpp:1757
            data.1.push((tr.special_side() & 0b10) != 0); // TriangleSelector.cpp:1758
            // Now save all children. Serialized in reverse order for compatibility with PrusaSlicer 2.3.1.
            let children = tr.children;
            for child_idx in (0..=split_sides).rev() {
                // TriangleSelector.cpp:1761
                self.serialize_recursive(children[child_idx as usize], data); // TriangleSelector.cpp:1762
            }
        } else {
            // In case this is leaf, we better save information about its state.
            let mut n = tr.get_state().0 as i32; // TriangleSelector.cpp:1765
            if n >= 3 {
                // TriangleSelector.cpp:1766
                data.1.extend_from_slice(&[true, true]); // TriangleSelector.cpp:1767
                n -= 3; // TriangleSelector.cpp:1768
                while n >= 15 {
                    // TriangleSelector.cpp:1769
                    data.1.extend_from_slice(&[true, true, true, true]); // TriangleSelector.cpp:1770
                    n -= 15; // TriangleSelector.cpp:1771
                }
                for bit_idx in 0..4u64 {
                    // TriangleSelector.cpp:1774
                    data.1.push((n & (0b0001u64 << bit_idx) as i32) != 0); // TriangleSelector.cpp:1775
                }
            } else {
                // Simple case, compatible with PrusaSlicer 2.3.1 and older.
                data.1.push((n & 0b01) != 0); // TriangleSelector.cpp:1779
                data.1.push((n & 0b10) != 0); // TriangleSelector.cpp:1780
            }
        }
    }

    // ============================================================================
    // TriangleSelector.cpp:1801-1936 — deserialize
    // ============================================================================

    // void deserialize(const ...&data, bool needs_reset, EnforcerBlockerType max_ebt, EnforcerBlockerType to_delete_filament, EnforcerBlockerType replace_filament)
    // TriangleSelector.cpp:2115-2254
    //
    // Two faithful gates (both on unless set to 0):
    // - `TS_PAINT_STREAM_GUARD`: the pin's bounds-checked nibble reads (`read_paint_nibble`, a
    //   truncated or out-of-range stream stops that root instead of indexing past the bits) and its
    //   int state decoding with `clamp_paint_state` (a state that wraps the int8_t storage, or
    //   lies outside [0, max_ebt], becomes NONE). Off: the earlier code, which indexes the bits
    //   unchecked and decodes into int8_t. Both arms agree on every well-formed stream.
    // - `EXTRUDER_MAX_PIN`: `max_ebt == EnforcerBlockerType::EXTRUDER_MAX` (what callers pass for
    //   the C++ default argument `ExtruderMax`) is read as the pin's `ExtruderMax` (96).
    pub fn deserialize(&mut self, data: &SerializedData, needs_reset: bool, max_ebt: EnforcerBlockerType, to_delete_filament: EnforcerBlockerType, replace_filament: EnforcerBlockerType) {
        let stream_guard = crate::faithful_gate("TS_PAINT_STREAM_GUARD");
        let max_ebt = if max_ebt == EnforcerBlockerType::EXTRUDER_MAX { EnforcerBlockerType::extruder_max() } else { max_ebt };
        if needs_reset {
            self.reset(); // dump any current state  // TriangleSelector.cpp:2122
        }
        for &(triangle_id, _ibit) in &data.0 {
            // TriangleSelector.cpp:2123
            if triangle_id >= self.m_triangles.len() as i32 {
                // TriangleSelector.cpp:2124
                crate::boost_log_trivial!(Info, "array bound:error:triangle_id >= int(m_triangles.size())"); // TriangleSelector.cpp:2125
                return; // TriangleSelector.cpp:2126
            }
        }
        // Reserve number of triangles as if each triangle was saved with 4 bits.
        // With MMU painting this estimate may be somehow low, but better than nothing.
        let its = self.its();
        self.m_triangles.reserve(its.indices.len().max(data.1.len() / 4)); // TriangleSelector.cpp:2131
        // Number of triangles is twice the number of vertices on a large manifold mesh of genus zero.
        self.m_vertices.reserve(its.vertices.len().max(self.m_triangles.len() / 2)); // TriangleSelector.cpp:2134

        // Vector to store all parents that have offsprings.  // TriangleSelector.cpp:2137-2142
        #[derive(Clone, Copy)]
        struct ProcessingInfo {
            facet_id: i32,
            neighbors: Vec3i,
            processed_children: i32,
            total_children: i32,
        }
        // Depth-first queue of a source mesh triangle and its childern.
        let mut parents: Vec<ProcessingInfo> = Vec::new(); // TriangleSelector.cpp:2145

        for &(triangle_id, ibit0) in &data.0 {
            // TriangleSelector.cpp:2147
            debug_assert!(triangle_id < self.m_triangles.len() as i32); // TriangleSelector.cpp:2148
            let mut stream_ok = true; // TriangleSelector.cpp:2149
            let mut ibit = ibit0;
            // next_nibble lambda (TriangleSelector.cpp:2151-2156)
            macro_rules! next_nibble {
                () => {{
                    let mut n = 0i32;
                    if stream_guard {
                        if !read_paint_nibble(&data.1, &mut ibit, &mut n) {
                            stream_ok = false;
                        }
                    } else {
                        // TS_PAINT_STREAM_GUARD=0: the earlier unchecked read.
                        for i in 0..4 {
                            n |= (data.1[ibit as usize] as i32) << i;
                            ibit += 1;
                        }
                    }
                    n
                }};
            }

            parents.clear(); // TriangleSelector.cpp:2158
            while stream_ok {
                // TriangleSelector.cpp:2159
                // Read next triangle info.
                let code = next_nibble!(); // TriangleSelector.cpp:2161
                if !stream_ok {
                    break; // TriangleSelector.cpp:2162-2163
                }
                let num_of_split_sides = code & 0b11; // TriangleSelector.cpp:2164
                let num_of_children = if num_of_split_sides == 0 { 0 } else { num_of_split_sides + 1 }; // TriangleSelector.cpp:2165
                let is_split = num_of_children != 0; // TriangleSelector.cpp:2166
                // Only valid if not is_split. Value of the second nibble was subtracted by 3, so it is added back.
                // Decode as int first: EnforcerBlockerType is int8_t, so a huge 0b1111-run wraps negative and skips max_ebt.
                let mut state_i: i32 = 0; // TriangleSelector.cpp:2169
                if !is_split {
                    // TriangleSelector.cpp:2170
                    if (code & 0b1100) == 0b1100 {
                        // TriangleSelector.cpp:2171
                        let mut next_code = next_nibble!(); // TriangleSelector.cpp:2172
                        let mut num: i32 = 0; // TriangleSelector.cpp:2173
                        while stream_ok && next_code == 0b1111 {
                            // TriangleSelector.cpp:2174
                            num += 1; // TriangleSelector.cpp:2175
                            next_code = next_nibble!(); // TriangleSelector.cpp:2176
                        }
                        if !stream_ok {
                            break; // TriangleSelector.cpp:2178-2179
                        }
                        state_i = next_code.wrapping_add(num.wrapping_mul(15)).wrapping_add(3); // TriangleSelector.cpp:2180
                    } else {
                        state_i = code >> 2; // TriangleSelector.cpp:2183
                    }
                }

                let state = if stream_guard {
                    // BBS
                    if state_i == to_delete_filament.0 as i32 {
                        // TriangleSelector.cpp:2188
                        state_i = replace_filament.0 as i32; // TriangleSelector.cpp:2189
                    } else if to_delete_filament != EnforcerBlockerType::NONE && state_i != 0 {
                        // TriangleSelector.cpp:2190
                        if state_i > to_delete_filament.0 as i32 {
                            state_i -= 1; // TriangleSelector.cpp:2191-2192
                        }
                    }
                    clamp_paint_state(state_i, max_ebt) // TriangleSelector.cpp:2195
                } else {
                    // TS_PAINT_STREAM_GUARD=0: the earlier decoding into int8_t.
                    let mut state = EnforcerBlockerType(state_i as i8);
                    if state == to_delete_filament {
                        state = replace_filament;
                    } else if to_delete_filament != EnforcerBlockerType::NONE && state != EnforcerBlockerType::NONE {
                        state = if state > to_delete_filament { EnforcerBlockerType((state.0 as i32 - 1) as i8) } else { state };
                    }
                    if state > max_ebt {
                        debug_assert!(false);
                        state = EnforcerBlockerType::NONE;
                    }
                    state
                };

                // Only valid if is_split.
                let special_side = code >> 2; // TriangleSelector.cpp:2198

                // Take care of the first iteration separately, so handling of the others is simpler.
                if parents.is_empty() {
                    // TriangleSelector.cpp:2201
                    if is_split {
                        // root is split, add it into list of parents and split it.
                        // then go to the next.
                        let neighbors = self.m_neighbors[triangle_id as usize]; // TriangleSelector.cpp:2205
                        parents.push(ProcessingInfo { facet_id: triangle_id, neighbors, processed_children: 0, total_children: num_of_children }); // TriangleSelector.cpp:2206
                        self.m_triangles[triangle_id as usize].set_division(num_of_split_sides, special_side); // TriangleSelector.cpp:2207
                        self.perform_split(triangle_id, &neighbors, EnforcerBlockerType::NONE); // TriangleSelector.cpp:2208
                        continue; // TriangleSelector.cpp:2209
                    } else {
                        // root is not split. just set the state and that's it.
                        self.m_triangles[triangle_id as usize].set_state(state); // TriangleSelector.cpp:2212
                        break; // TriangleSelector.cpp:2213
                    }
                }

                // This is not the first iteration. This triangle is a child of last seen parent.
                debug_assert!(!parents.is_empty()); // TriangleSelector.cpp:2218
                debug_assert!(parents.last().unwrap().processed_children < parents.last().unwrap().total_children); // TriangleSelector.cpp:2219

                if is_split {
                    // TriangleSelector.cpp:2221
                    // split the triangle and save it as parent of the next ones.
                    let last = *parents.last().unwrap();
                    let tr = self.m_triangles[last.facet_id as usize].clone(); // TriangleSelector.cpp:2223
                    let child_idx = last.total_children - last.processed_children - 1; // TriangleSelector.cpp:2224
                    let neighbors = self.child_neighbors(&tr, &last.neighbors, child_idx); // TriangleSelector.cpp:2225
                    let this_idx = tr.children[child_idx as usize]; // TriangleSelector.cpp:2226
                    self.m_triangles[this_idx as usize].set_division(num_of_split_sides, special_side); // TriangleSelector.cpp:2227
                    self.perform_split(this_idx, &neighbors, EnforcerBlockerType::NONE); // TriangleSelector.cpp:2228
                    parents.push(ProcessingInfo { facet_id: this_idx, neighbors, processed_children: 0, total_children: num_of_children }); // TriangleSelector.cpp:2229
                } else {
                    // this triangle belongs to last split one
                    let last = *parents.last().unwrap();
                    let child_idx = last.total_children - last.processed_children - 1; // TriangleSelector.cpp:2232
                    let c = self.m_triangles[last.facet_id as usize].children[child_idx as usize];
                    self.m_triangles[c as usize].set_state(state); // TriangleSelector.cpp:2233
                    parents.last_mut().unwrap().processed_children += 1; // TriangleSelector.cpp:2234
                }

                // If all children of the past parent triangle are claimed, move to grandparent.
                while parents.last().unwrap().processed_children == parents.last().unwrap().total_children {
                    // TriangleSelector.cpp:2238
                    parents.pop(); // TriangleSelector.cpp:2239

                    if parents.is_empty() {
                        break; // TriangleSelector.cpp:2241-2242
                    }

                    // And increment the grandparent children counter, because
                    // we have just finished that branch and got back here.
                    parents.last_mut().unwrap().processed_children += 1; // TriangleSelector.cpp:2246
                }

                // In case we popped back the root, we should be done.
                if parents.is_empty() {
                    break; // TriangleSelector.cpp:2250-2251
                }
            }
        }
    }

    // ============================================================================
    // TriangleSelector.cpp:2256-2328 — static has_facets
    // ============================================================================

    // Lightweight variant of deserialization, which only tests whether a face of test_state exists.
    // static bool has_facets(const ...&data, const EnforcerBlockerType test_state)  // TriangleSelector.cpp:2257-2328
    // Gate `TS_PAINT_STREAM_GUARD` as in `deserialize`: on, the pin's bounds-checked reads; off,
    // the earlier unchecked ones.
    pub fn has_facets_data(data: &SerializedData, test_state: EnforcerBlockerType) -> bool {
        let stream_guard = crate::faithful_gate("TS_PAINT_STREAM_GUARD");
        // Depth-first queue of a number of unvisited children.
        // Kept outside of the loop to avoid re-allocating inside the loop.
        let mut parents_children: Vec<i32> = Vec::with_capacity(64); // TriangleSelector.cpp:2261-2262

        for triangle_id_and_ibit in &data.0 {
            // TriangleSelector.cpp:2264
            let mut ibit = triangle_id_and_ibit.1; // TriangleSelector.cpp:2265
            let mut stream_ok = true; // TriangleSelector.cpp:2266
            // next_nibble lambda (TriangleSelector.cpp:2267-2272)
            macro_rules! next_nibble {
                () => {{
                    let mut n = 0i32;
                    if stream_guard {
                        if !read_paint_nibble(&data.1, &mut ibit, &mut n) {
                            stream_ok = false;
                        }
                    } else {
                        // TS_PAINT_STREAM_GUARD=0: the earlier unchecked read.
                        for i in 0..4 {
                            n |= (data.1[ibit as usize] as i32) << i;
                            ibit += 1;
                        }
                    }
                    n
                }};
            }
            // < 0 -> negative of a number of children
            // >= 0 -> state
            // num_children_or_state lambda (TriangleSelector.cpp:2275-2299)
            macro_rules! num_children_or_state {
                () => {{
                    let code = next_nibble!(); // TriangleSelector.cpp:2276
                    if !stream_ok {
                        0 // TriangleSelector.cpp:2277-2278
                    } else {
                        let num_of_split_sides = code & 0b11; // TriangleSelector.cpp:2279
                        if num_of_split_sides == 0 {
                            // TriangleSelector.cpp:2280
                            let mut st: i32 = 0; // TriangleSelector.cpp:2281
                            let mut ok = true;
                            if (code & 0b1100) == 0b1100 {
                                // TriangleSelector.cpp:2282
                                let mut next_code = next_nibble!(); // TriangleSelector.cpp:2283
                                let mut num: i32 = 0; // TriangleSelector.cpp:2284
                                while stream_ok && next_code == 0b1111 {
                                    // TriangleSelector.cpp:2285
                                    num += 1; // TriangleSelector.cpp:2286
                                    next_code = next_nibble!(); // TriangleSelector.cpp:2287
                                }
                                if !stream_ok {
                                    ok = false; // TriangleSelector.cpp:2289-2290 — return 0;
                                } else {
                                    st = next_code.wrapping_add(num.wrapping_mul(15)).wrapping_add(3); // TriangleSelector.cpp:2291
                                }
                            } else {
                                st = code >> 2; // TriangleSelector.cpp:2293
                            }
                            if ok { st } else { 0 } // TriangleSelector.cpp:2295
                        } else {
                            -num_of_split_sides - 1 // < 0 -> negative of a number of children  // TriangleSelector.cpp:2297
                        }
                    }
                }};
            }

            let state = num_children_or_state!(); // TriangleSelector.cpp:2301
            if !stream_ok {
                continue; // TriangleSelector.cpp:2302-2303
            }
            if state < 0 {
                // Root is split.
                parents_children.clear(); // TriangleSelector.cpp:2306
                parents_children.push(-state); // TriangleSelector.cpp:2307
                loop {
                    // TriangleSelector.cpp:2308
                    let back = *parents_children.last().unwrap() - 1;
                    *parents_children.last_mut().unwrap() = back; // -- parents_children.back()  // TriangleSelector.cpp:2309
                    if back >= 0 {
                        let state = num_children_or_state!(); // TriangleSelector.cpp:2310
                        if !stream_ok {
                            break; // TriangleSelector.cpp:2311-2312
                        }
                        if state < 0 {
                            // Child is split.
                            parents_children.push(-state); // TriangleSelector.cpp:2315
                        } else if state == test_state.0 as i32 {
                            // Child is not split and a face of test_state was found.
                            return true; // TriangleSelector.cpp:2318
                        }
                    } else {
                        parents_children.pop(); // TriangleSelector.cpp:2320
                    }
                    if parents_children.is_empty() {
                        break; // TriangleSelector.cpp:2321
                    }
                }
            } else if state == test_state.0 as i32 {
                // Root is not split and a face of test_state was found.
                return true; // TriangleSelector.cpp:2324
            }
        }

        false // TriangleSelector.cpp:2327
    }

    // ============================================================================
    // TriangleSelector.cpp:2004-2036 — seed_fill_unselect_all / apply / shift_states_above
    // ============================================================================

    // void seed_fill_unselect_all_triangles()  // TriangleSelector.cpp:2004-2009
    pub fn seed_fill_unselect_all_triangles(&mut self) {
        for triangle in &mut self.m_triangles {
            // TriangleSelector.cpp:2006
            if !triangle.is_split() {
                triangle.unselect_by_seed_fill(); // TriangleSelector.cpp:2008
            }
        }
    }

    // void seed_fill_apply_on_triangles(EnforcerBlockerType new_state)  // TriangleSelector.cpp:2011-2022
    pub fn seed_fill_apply_on_triangles(&mut self, new_state: EnforcerBlockerType) {
        for triangle in &mut self.m_triangles {
            // TriangleSelector.cpp:2013
            if !triangle.is_split() && triangle.is_selected_by_seed_fill() {
                triangle.set_state(new_state); // TriangleSelector.cpp:2015
            }
        }

        for facet_idx in 0..self.m_triangles.len() {
            // TriangleSelector.cpp:2017-2019 — iterate, compute index from pointer diff.
            if self.m_triangles[facet_idx].is_split() && self.m_triangles[facet_idx].valid() {
                self.remove_useless_children(facet_idx as i32); // TriangleSelector.cpp:2020
            }
        }
    }

    /// TriangleSelector.cpp:2350-2358 `void fill_unpainted(EnforcerBlockerType state)`: set every
    /// unpainted (NONE) triangle to the given state, keeping painted detail (TriangleSelector.hpp:373-378).
    pub fn fill_unpainted(&mut self, state: EnforcerBlockerType) {
        for triangle in &mut self.m_triangles {
            // TriangleSelector.cpp:2352
            if triangle.is_split() || !triangle.valid() {
                continue; // TriangleSelector.cpp:2353-2354
            }
            if triangle.get_state() == EnforcerBlockerType::NONE {
                triangle.set_state(state); // TriangleSelector.cpp:2355-2356
            }
        }
    }

    // void shift_states_above(EnforcerBlockerType threshold, int delta)  // TriangleSelector.cpp:2360-2372
    pub fn shift_states_above(&mut self, threshold: EnforcerBlockerType, delta: i32) {
        for triangle in &mut self.m_triangles {
            // TriangleSelector.cpp:2026
            if triangle.is_split() || !triangle.valid() {
                continue; // TriangleSelector.cpp:2027-2028
            }
            let s = triangle.get_state(); // TriangleSelector.cpp:2029
            if s >= threshold && s != EnforcerBlockerType::NONE {
                // TriangleSelector.cpp:2030
                let new_val = s.0 as i32 + delta; // TriangleSelector.cpp:2031
                if new_val >= 0 {
                    triangle.set_state(EnforcerBlockerType(new_val as i8)); // TriangleSelector.cpp:2033
                }
            }
        }
    }

    /// TriangleSelector.cpp:2374-2382 `void replace_state(EnforcerBlockerType from, EnforcerBlockerType to)`:
    /// repaint every triangle in state `from` with state `to`; other states are left untouched.
    pub fn replace_state(&mut self, from: EnforcerBlockerType, to: EnforcerBlockerType) {
        for triangle in &mut self.m_triangles {
            // TriangleSelector.cpp:2376
            if triangle.is_split() || !triangle.valid() {
                continue; // TriangleSelector.cpp:2377-2378
            }
            if triangle.get_state() == from {
                triangle.set_state(to); // TriangleSelector.cpp:2379-2380
            }
        }
    }
}
