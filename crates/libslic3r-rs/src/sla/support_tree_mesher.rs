//! Faithful 1:1 port of `SLA/SupportTreeMesher.{hpp,cpp}` from BambuStudio libslic3r.
//!
//! C++ Reference:
//! - src/libslic3r/SLA/SupportTreeMesher.hpp (130 lines)
//! - src/libslic3r/SLA/SupportTreeMesher.cpp (271 lines)
//!
//! Fidelity notes (byte-exact G-code parity):
//! - Mesh vertices are `Vec3f` (`f32`) and triangle indices `Vec3i` (`i32`),
//!   matching C++ `indexed_triangle_set` (admesh). All geometry math is done in
//!   `f64` and narrowed to `f32` exactly where the C++ does (`.cast<float>()`,
//!   `float(...)` constructor arguments, `float += double` compound ops).
//! - `coord_t` index arithmetic is carried in `i64` and narrowed to `i32` on
//!   `emplace_back` into `indices`, mirroring the C++ `Vec3i` narrowing.
//! - The C++ `get_mesh` overload set (Head/Pillar/Pedestal/Junction/Bridge/
//!   DiffBridge) maps to one Rust function per overload: `get_mesh_head`,
//!   `get_mesh_pillar`, `get_mesh_pedestal`, `get_mesh_junction`,
//!   `get_mesh_bridge`, `get_mesh_diff_bridge`.
//! - `Eigen::Quaternion<float>::FromTwoVectors` and the quaternion-vector
//!   product are reproduced from Eigen (Quaternion.h `setFromTwoVectors`,
//!   `_transformVector`) in `f32`. See the documented divergence in
//!   `QuaternionF::from_two_vectors` for the (never hit in practice)
//!   antiparallel branch.
//! - C++ default arguments (`portion`, `fa`, `steps = 45`, `sp = Vec3d::Zero()`)
//!   cannot be expressed in Rust; callers pass them explicitly.

use crate::geometry::Vec3d;
use crate::libslic3r::EPSILON;
use crate::normal_utils::indexed_triangle_set;
use crate::sla::support_tree_builder::{Bridge, DiffBridge, Head, Junction, Pedestal, Pillar};
use crate::triangle_mesh::{its_merge, Vec3f, Vec3i};
use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Local mirrors of the Eigen primitives used by this translation unit.
// ---------------------------------------------------------------------------

/// Eigen `Rotation2Dd(angle) * Eigen::Vector2d(x, y)`: rotates by
/// `[cos -sin; sin cos] * v` in double precision.
#[inline]
fn rotate2d_d(angle: f64, v: (f64, f64)) -> (f64, f64) {
    let c = angle.cos();
    let s = angle.sin();
    (c * v.0 - s * v.1, s * v.0 + c * v.1)
}

/// `Vec3d::cast<float>()` — per-component `f64 -> f32` narrowing, as Eigen does.
#[inline]
fn cast_f32(v: &Vec3d) -> Vec3f {
    Vec3f::new(v.x as f32, v.y as f32, v.z as f32)
}

/// Eigen `MatrixBase::unitOrthogonal()` for a 3-vector of `f32`
/// (Eigen/src/Geometry/OrthoMethods.h, `unitOrthogonal_selector<Derived,3>`).
fn unit_orthogonal(src: &Vec3f) -> Vec3f {
    // internal::isMuchSmallerThan(x, y) == (|x| <= |y| * prec) with
    // NumTraits<float>::dummy_precision() == 1e-5f.
    let prec = 1e-5f32;
    let not_much_smaller = |x: f32, y: f32| !(x.abs() <= y.abs() * prec);
    /* Let us compute the crossed product of *this with a vector
     * that is not too close to being colinear to *this.
     */
    if not_much_smaller(src.x, src.z) || not_much_smaller(src.y, src.z) {
        let invnm = 1.0f32 / (src.x * src.x + src.y * src.y).sqrt();
        Vec3f::new(-src.y * invnm, src.x * invnm, 0.0)
    } else {
        let invnm = 1.0f32 / (src.y * src.y + src.z * src.z).sqrt();
        Vec3f::new(0.0, -src.z * invnm, src.y * invnm)
    }
}

/// Minimal mirror of `Eigen::Quaternion<float>` providing exactly the two
/// operations SupportTreeMesher.hpp uses: `FromTwoVectors` and `q * v`.
#[derive(Debug, Clone, Copy)]
struct QuaternionF {
    /// Scalar part `w`.
    w: f32,
    /// Vector part `(x, y, z)`.
    x: f32,
    y: f32,
    z: f32,
}

impl QuaternionF {
    /// `Eigen::Quaternion<float>::FromTwoVectors(a, b)`
    /// (Eigen/src/Geometry/Quaternion.h, `setFromTwoVectors`).
    fn from_two_vectors(a: Vec3f, b: Vec3f) -> Self {
        // Quaternion.h: Vector3 v0 = a.normalized();
        let v0 = a.normalize();
        // Quaternion.h: Vector3 v1 = b.normalized();
        let v1 = b.normalize();
        // Quaternion.h: Scalar c = v1.dot(v0);
        let c = v1.dot(&v0);

        // if dot == -1, vectors are nearly opposites
        // Quaternion.h: if (c < Scalar(-1) + NumTraits<Scalar>::dummy_precision())
        if c < -1.0f32 + 1e-5f32 {
            // DIVERGENCE (documented): Eigen computes the rotation axis as the
            // null-space vector of the 2x3 matrix [v0^T; v1^T] via JacobiSVD
            // (`svd.matrixV().col(2)`). Porting JacobiSVD is out of scope, so
            // the axis is chosen with Eigen's own deterministic
            // `unitOrthogonal()` construction instead. Both yield a valid unit
            // axis orthogonal to v0; the resulting 180-degree rotations can
            // differ by a roll about v0. This branch only triggers when the
            // two directions are within ~1e-5 of exactly antiparallel (a
            // support element pointing exactly opposite its reference axis).
            let c = c.max(-1.0f32);
            let axis = unit_orthogonal(&v0);
            // Quaternion.h: Scalar w2 = (Scalar(1)+c)*Scalar(0.5);
            let w2 = (1.0f32 + c) * 0.5f32;
            // Quaternion.h: this->w() = sqrt(w2);
            // Quaternion.h: this->vec() = axis * sqrt(Scalar(1) - w2);
            let s = (1.0f32 - w2).sqrt();
            return Self {
                w: w2.sqrt(),
                x: axis.x * s,
                y: axis.y * s,
                z: axis.z * s,
            };
        }
        // Quaternion.h: Vector3 axis = v0.cross(v1);
        let axis = v0.cross(&v1);
        // Quaternion.h: Scalar s = sqrt((Scalar(1)+c)*Scalar(2));
        let s = ((1.0f32 + c) * 2.0f32).sqrt();
        // Quaternion.h: Scalar invs = Scalar(1)/s;
        let invs = 1.0f32 / s;
        // Quaternion.h: this->vec() = axis * invs; this->w() = s * Scalar(0.5);
        Self {
            w: s * 0.5f32,
            x: axis.x * invs,
            y: axis.y * invs,
            z: axis.z * invs,
        }
    }

    /// Eigen `q * v` (Quaternion.h, `QuaternionBase::_transformVector`):
    /// ```text
    /// Vector3 uv = this->vec().cross(v);
    /// uv += uv;
    /// return v + this->w() * uv + this->vec().cross(uv);
    /// ```
    fn transform_vector(&self, v: Vec3f) -> Vec3f {
        let vec = Vec3f::new(self.x, self.y, self.z);
        let mut uv = vec.cross(&v);
        uv += uv;
        v + uv * self.w + vec.cross(&uv)
    }
}

// ---------------------------------------------------------------------------
// SupportTreeMesher.hpp
// ---------------------------------------------------------------------------

/// `using Portion = std::tuple<double, double>;`
/// SupportTreeMesher.hpp:12
pub type Portion = (f64, f64);

/// SupportTreeMesher.hpp:14-17
/// C++: `inline Portion make_portion(double a, double b)`
#[inline]
pub fn make_portion(a: f64, b: f64) -> Portion {
    // SupportTreeMesher.hpp:16
    (a, b)
}

// ---------------------------------------------------------------------------
// SupportTreeMesher.cpp
// ---------------------------------------------------------------------------

/// SupportTreeMesher.cpp:5-94
/// C++: `indexed_triangle_set sphere(double rho, Portion portion, double fa)`
/// (defaults: `portion = make_portion(0., 2. * PI)`, `fa = 2. * PI / 360.` —
/// SupportTreeMesher.hpp:19-21; callers pass them explicitly in Rust.)
pub fn sphere(rho: f64, portion: Portion, fa: f64) -> indexed_triangle_set {
    // SupportTreeMesher.cpp:7
    let mut ret = indexed_triangle_set::default();

    // prohibit close to zero radius
    // SupportTreeMesher.cpp:10
    if rho <= 1e-6 && rho >= -1e-6 {
        return ret;
    }

    // SupportTreeMesher.cpp:12-13
    // C++: auto& vertices = ret.vertices; auto& facets = ret.indices;
    // (Rust borrow rules: accessed as ret.vertices / ret.indices below.)

    // Algorithm:
    // Add points one-by-one to the sphere grid and form facets using relative
    // coordinates. Sphere is composed effectively of a mesh of stacked circles.

    // adjust via rounding to get an even multiple for any provided angle.
    // SupportTreeMesher.cpp:20
    let angle = 2.0 * PI / (2.0 * PI / fa).floor();

    // Ring to be scaled to generate the steps of the sphere
    // SupportTreeMesher.cpp:23-25
    let mut ring: Vec<f64> = Vec::new();
    let mut i = 0.0f64;
    while i < 2.0 * PI {
        ring.push(i);
        i += angle;
    }

    // SupportTreeMesher.cpp:27-28
    let sbegin = (2.0 * portion.0 / angle) as usize;
    let send = (2.0 * portion.1 / angle) as usize;

    // SupportTreeMesher.cpp:30-31
    let steps = ring.len();
    let increment = 1.0 / steps as f64;

    // special case: first ring connects to 0,0,0
    // insert and form facets.
    // SupportTreeMesher.cpp:35-37
    if sbegin == 0 {
        ret.vertices.push(Vec3f::new(
            0.0,
            0.0,
            (-rho + increment * sbegin as f64 * 2.0 * rho) as f32,
        ));
    }

    // SupportTreeMesher.cpp:39
    let mut id = ret.vertices.len() as i64; // coord_t
    // SupportTreeMesher.cpp:40
    for i in 0..ring.len() {
        // Fixed scaling
        // SupportTreeMesher.cpp:42
        let z = -rho + increment * rho * 2.0 * (sbegin as f64 + 1.0);
        // radius of the circle for this step.
        // SupportTreeMesher.cpp:44
        let r = (rho * rho - z * z).abs().sqrt();
        // SupportTreeMesher.cpp:45
        let b = rotate2d_d(ring[i], (0.0, r));
        // SupportTreeMesher.cpp:46
        ret.vertices
            .push(Vec3f::new(b.0 as f32, b.1 as f32, z as f32));

        // SupportTreeMesher.cpp:48-50
        if sbegin == 0 {
            if i == 0 {
                ret.indices.push(Vec3i::new(ring.len() as i32, 0, 1));
            } else {
                ret.indices.push(Vec3i::new((id - 1) as i32, 0, id as i32));
            }
        }
        // SupportTreeMesher.cpp:51
        id += 1;
    }

    // General case: insert and form facets for each step,
    // joining it to the ring below it.
    // SupportTreeMesher.cpp:56
    // (`send - 1` on a C++ size_t wraps for send == 0; mirrored with
    //  wrapping_sub.)
    let mut s = sbegin + 2;
    while s < send.wrapping_sub(1) {
        // SupportTreeMesher.cpp:57
        let z = -rho + increment * (s as f64 * 2.0 * rho);
        // SupportTreeMesher.cpp:58
        let r = (rho * rho - z * z).abs().sqrt();

        // SupportTreeMesher.cpp:60
        for i in 0..ring.len() {
            // SupportTreeMesher.cpp:61
            let b = rotate2d_d(ring[i], (0.0, r));
            // SupportTreeMesher.cpp:62
            ret.vertices
                .push(Vec3f::new(b.0 as f32, b.1 as f32, z as f32));
            // SupportTreeMesher.cpp:63
            let id_ringsize = id - ring.len() as i64; // coord_t(id - int(ring.size()))
            if i == 0 {
                // wrap around
                // SupportTreeMesher.cpp:66
                ret.indices.push(Vec3i::new(
                    (id - 1) as i32,
                    id as i32,
                    (id + ring.len() as i64 - 1) as i32,
                ));
                // SupportTreeMesher.cpp:67
                ret.indices
                    .push(Vec3i::new((id - 1) as i32, id_ringsize as i32, id as i32));
            } else {
                // SupportTreeMesher.cpp:69
                ret.indices.push(Vec3i::new(
                    (id_ringsize - 1) as i32,
                    id_ringsize as i32,
                    id as i32,
                ));
                // SupportTreeMesher.cpp:70
                ret.indices.push(Vec3i::new(
                    (id - 1) as i32,
                    (id_ringsize - 1) as i32,
                    id as i32,
                ));
            }
            // SupportTreeMesher.cpp:72
            id += 1;
        }
        s += 1;
    }

    // special case: last ring connects to 0,0,rho*2.0
    // only form facets.
    // SupportTreeMesher.cpp:78
    if send >= (2.0 * PI / angle) as usize {
        // SupportTreeMesher.cpp:79
        ret.vertices.push(Vec3f::new(
            0.0,
            0.0,
            (-rho + increment * send as f64 * 2.0 * rho) as f32,
        ));
        // SupportTreeMesher.cpp:80
        for i in 0..ring.len() {
            // SupportTreeMesher.cpp:81
            let id_ringsize = id - ring.len() as i64;
            if i == 0 {
                // third vertex is on the other side of the ring.
                // SupportTreeMesher.cpp:84
                ret.indices
                    .push(Vec3i::new((id - 1) as i32, id_ringsize as i32, id as i32));
            } else {
                // SupportTreeMesher.cpp:86
                let ci = id_ringsize + i as i64; // coord_t(id_ringsize + coord_t(i))
                // SupportTreeMesher.cpp:87
                ret.indices
                    .push(Vec3i::new((ci - 1) as i32, ci as i32, id as i32));
            }
        }
    }
    // SupportTreeMesher.cpp:91 — `id++;` (no further use; no effect)

    // SupportTreeMesher.cpp:93
    ret
}

/// SupportTreeMesher.cpp:96-156
/// C++: `indexed_triangle_set cylinder(double r, double h, size_t ssteps, const Vec3d &sp)`
/// Down facing cylinder in Z direction with arguments:
/// r: radius
/// h: Height
/// ssteps: how many edges will create the base circle
/// sp: starting point
/// (defaults: `steps = 45`, `sp = Vec3d::Zero()` — SupportTreeMesher.hpp:28-31.)
pub fn cylinder(r: f64, h: f64, ssteps: usize, sp: &Vec3d) -> indexed_triangle_set {
    // SupportTreeMesher.cpp:98
    debug_assert!(ssteps > 0);

    // SupportTreeMesher.cpp:100
    let mut ret = indexed_triangle_set::default();

    // SupportTreeMesher.cpp:102
    let steps = ssteps as i32;
    // SupportTreeMesher.cpp:103-105
    // C++: auto& points = ret.vertices; auto& indices = ret.indices;
    ret.vertices.reserve(2 * ssteps);
    // SupportTreeMesher.cpp:106
    let a = 2.0 * PI / steps as f64;

    // SupportTreeMesher.cpp:108
    let jp = *sp;
    // SupportTreeMesher.cpp:109
    let endp = Vec3d::new(sp.x(), sp.y(), sp.z() + h);

    // Upper circle points
    // SupportTreeMesher.cpp:112-117
    for i in 0..steps {
        let phi = i as f64 * a;
        let ex = (endp.x() + r * phi.cos()) as f32;
        let ey = (endp.y() + r * phi.sin()) as f32;
        ret.vertices.push(Vec3f::new(ex, ey, endp.z() as f32));
    }

    // Lower circle points
    // SupportTreeMesher.cpp:120-125
    for i in 0..steps {
        let phi = i as f64 * a;
        let x = (jp.x() + r * phi.cos()) as f32;
        let y = (jp.y() + r * phi.sin()) as f32;
        ret.vertices.push(Vec3f::new(x, y, jp.z() as f32));
    }

    // Now create long triangles connecting upper and lower circles
    // SupportTreeMesher.cpp:128
    ret.indices.reserve(2 * ssteps);
    // SupportTreeMesher.cpp:129
    let offs = steps;
    // SupportTreeMesher.cpp:130-133
    for i in 0..steps - 1 {
        ret.indices.push(Vec3i::new(i, i + offs, offs + i + 1));
        ret.indices.push(Vec3i::new(i, offs + i + 1, i + 1));
    }

    // Last triangle connecting the first and last vertices
    // SupportTreeMesher.cpp:136-138
    let last = steps - 1;
    ret.indices.push(Vec3i::new(0, last, offs));
    ret.indices.push(Vec3i::new(last, offs + last, offs));

    // According to the slicing algorithms, we need to aid them with generating
    // a watertight body. So we create a triangle fan for the upper and lower
    // ending of the cylinder to close the geometry.
    // SupportTreeMesher.cpp:143
    ret.vertices.push(cast_f32(&jp));
    let mut ci = ret.vertices.len() as i32 - 1;
    // SupportTreeMesher.cpp:144-145
    for i in 0..steps - 1 {
        ret.indices.push(Vec3i::new(i + offs + 1, i + offs, ci));
    }

    // SupportTreeMesher.cpp:147
    ret.indices.push(Vec3i::new(offs, steps + offs - 1, ci));

    // SupportTreeMesher.cpp:149
    ret.vertices.push(cast_f32(&endp));
    ci = ret.vertices.len() as i32 - 1;
    // SupportTreeMesher.cpp:150-151
    for i in 0..steps - 1 {
        ret.indices.push(Vec3i::new(ci, i, i + 1));
    }

    // SupportTreeMesher.cpp:153
    ret.indices.push(Vec3i::new(steps - 1, 0, ci));

    // SupportTreeMesher.cpp:155
    ret
}

/// SupportTreeMesher.cpp:158-217
/// C++: `indexed_triangle_set pinhead(double r_pin, double r_back, double length, size_t steps)`
/// (default: `steps = 45` — SupportTreeMesher.hpp:33-36.)
pub fn pinhead(r_pin: f64, r_back: f64, length: f64, steps: usize) -> indexed_triangle_set {
    // SupportTreeMesher.cpp:163-166
    debug_assert!(steps > 0);
    debug_assert!(length >= 0.0);
    debug_assert!(r_back > 0.0);
    debug_assert!(r_pin > 0.0);

    // SupportTreeMesher.cpp:168
    let mut mesh = indexed_triangle_set::default();

    // We create two spheres which will be connected with a robe that fits
    // both circles perfectly.

    // Set up the model detail level
    // SupportTreeMesher.cpp:174
    let detail = 2.0 * PI / steps as f64;

    // We don't generate whole circles. Instead, we generate only the
    // portions which are visible (not covered by the robe) To know the
    // exact portion of the bottom and top circles we need to use some
    // rules of tangent circles from which we can derive (using simple
    // triangles the following relations:

    // The height of the whole mesh
    // SupportTreeMesher.cpp:183
    let h = r_back + r_pin + length;
    // SupportTreeMesher.cpp:184
    let phi = PI / 2.0 - ((r_back - r_pin) / h).acos();

    // To generate a whole circle we would pass a portion of (0, Pi)
    // To generate only a half horizontal circle we can pass (0, Pi/2)
    // The calculated phi is an offset to the half circles needed to smooth
    // the transition from the circle to the robe geometry

    // SupportTreeMesher.cpp:191
    let s1 = sphere(r_back, make_portion(0.0, PI / 2.0 + phi), detail);
    // SupportTreeMesher.cpp:192
    let mut s2 = sphere(r_pin, make_portion(PI / 2.0 + phi, PI), detail);

    // SupportTreeMesher.cpp:194
    // C++: for (auto &p : s2.vertices) p.z() += h;
    // (float += double: computed in double, truncated back to float.)
    for p in &mut s2.vertices {
        p.z = (p.z as f64 + h) as f32;
    }

    // SupportTreeMesher.cpp:196-197
    its_merge(&mut mesh, &s1);
    its_merge(&mut mesh, &s2);

    // SupportTreeMesher.cpp:199-206
    let mut idx1 = s1.vertices.len() - steps;
    let mut idx2 = s1.vertices.len();
    while idx1 < s1.vertices.len() - 1 {
        // SupportTreeMesher.cpp:201
        let i1s1 = idx1 as i64; // coord_t
        let i1s2 = idx2 as i64;
        // SupportTreeMesher.cpp:202
        let i2s1 = i1s1 + 1;
        let i2s2 = i1s2 + 1;

        // SupportTreeMesher.cpp:204-205
        mesh.indices
            .push(Vec3i::new(i1s1 as i32, i2s1 as i32, i2s2 as i32));
        mesh.indices
            .push(Vec3i::new(i1s1 as i32, i2s2 as i32, i1s2 as i32));
        idx1 += 1;
        idx2 += 1;
    }

    // SupportTreeMesher.cpp:208-211
    let i1s1 = s1.vertices.len() as i64 - steps as i64;
    let i2s1 = s1.vertices.len() as i64 - 1;
    let i1s2 = s1.vertices.len() as i64;
    let i2s2 = s1.vertices.len() as i64 + steps as i64 - 1;

    // SupportTreeMesher.cpp:213-214
    mesh.indices
        .push(Vec3i::new(i2s2 as i32, i2s1 as i32, i1s1 as i32));
    mesh.indices
        .push(Vec3i::new(i1s2 as i32, i2s2 as i32, i1s1 as i32));

    // SupportTreeMesher.cpp:216
    mesh
}

/// SupportTreeMesher.cpp:219-268
/// C++: `indexed_triangle_set halfcone(double baseheight, double r_bottom,
///       double r_top, const Vec3d &pos, size_t steps)`
/// (defaults: `pt = Vec3d::Zero()`, `steps = 45` — SupportTreeMesher.hpp:38-42.)
pub fn halfcone(
    baseheight: f64,
    r_bottom: f64,
    r_top: f64,
    pos: &Vec3d,
    steps: usize,
) -> indexed_triangle_set {
    // SupportTreeMesher.cpp:225
    debug_assert!(steps > 0);

    // SupportTreeMesher.cpp:227
    if baseheight <= 0.0 || steps == 0 {
        return indexed_triangle_set::default();
    }

    // SupportTreeMesher.cpp:229
    let mut base = indexed_triangle_set::default();

    // SupportTreeMesher.cpp:231
    let a = 2.0 * PI / steps as f64;
    // SupportTreeMesher.cpp:232
    let last = steps as i32 - 1;
    // SupportTreeMesher.cpp:233
    let ep = Vec3d::new(pos.x(), pos.y(), pos.z() + baseheight);
    // SupportTreeMesher.cpp:234-239
    for i in 0..steps {
        let phi = i as f64 * a;
        let x = (pos.x() + r_top * phi.cos()) as f32;
        let y = (pos.y() + r_top * phi.sin()) as f32;
        base.vertices.push(Vec3f::new(x, y, ep.z() as f32));
    }

    // SupportTreeMesher.cpp:241-246
    for i in 0..steps {
        let phi = i as f64 * a;
        let x = (pos.x() + r_bottom * phi.cos()) as f32;
        let y = (pos.y() + r_bottom * phi.sin()) as f32;
        base.vertices.push(Vec3f::new(x, y, pos.z() as f32));
    }

    // SupportTreeMesher.cpp:248-249
    base.vertices.push(cast_f32(pos));
    base.vertices.push(cast_f32(&ep));

    // SupportTreeMesher.cpp:251-254
    // C++: auto& indices = base.indices;
    let hcenter = base.vertices.len() as i32 - 1;
    let lcenter = base.vertices.len() as i32 - 2;
    let offs = steps as i32;
    // SupportTreeMesher.cpp:255-260
    for i in 0..last {
        base.indices.push(Vec3i::new(i, i + offs, offs + i + 1));
        base.indices.push(Vec3i::new(i, offs + i + 1, i + 1));
        base.indices.push(Vec3i::new(i, i + 1, hcenter));
        base.indices.push(Vec3i::new(lcenter, offs + i + 1, offs + i));
    }

    // SupportTreeMesher.cpp:262-265
    base.indices.push(Vec3i::new(0, last, offs));
    base.indices.push(Vec3i::new(last, offs + last, offs));
    base.indices.push(Vec3i::new(hcenter, last, 0));
    base.indices.push(Vec3i::new(offs, offs + last, lcenter));

    // SupportTreeMesher.cpp:267
    base
}

// ---------------------------------------------------------------------------
// SupportTreeMesher.hpp inline get_mesh overloads
// ---------------------------------------------------------------------------

/// SupportTreeMesher.hpp:44-63
/// C++: `inline indexed_triangle_set get_mesh(const Head &h, size_t steps)`
pub fn get_mesh_head(h: &Head, steps: usize) -> indexed_triangle_set {
    // SupportTreeMesher.hpp:46
    let mut mesh = pinhead(h.r_pin_mm, h.r_back_mm, h.width_mm, steps);

    // SupportTreeMesher.hpp:48
    // C++: for (auto& p : mesh.vertices) p.z() -= (h.fullwidth() - h.r_back_mm);
    // (float -= double: computed in double, truncated back to float.)
    for p in &mut mesh.vertices {
        p.z = (p.z as f64 - (h.fullwidth() - h.r_back_mm)) as f32;
    }

    // SupportTreeMesher.hpp:50 — using Quaternion = Eigen::Quaternion<float>;

    // We rotate the head to the specified direction. The head's pointing
    // side is facing upwards so this means that it would hold a support
    // point with a normal pointing straight down. This is the reason of
    // the -1 z coordinate
    // SupportTreeMesher.hpp:56-57
    let quatern = QuaternionF::from_two_vectors(Vec3f::new(0.0, 0.0, -1.0), cast_f32(&h.dir));

    // SupportTreeMesher.hpp:59
    let pos = cast_f32(&h.pos);
    // SupportTreeMesher.hpp:60
    for p in &mut mesh.vertices {
        *p = quatern.transform_vector(*p) + pos;
    }

    // SupportTreeMesher.hpp:62
    mesh
}

/// SupportTreeMesher.hpp:65-74
/// C++: `inline indexed_triangle_set get_mesh(const Pillar &p, size_t steps)`
pub fn get_mesh_pillar(p: &Pillar, steps: usize) -> indexed_triangle_set {
    // SupportTreeMesher.hpp:67
    if p.height > EPSILON {
        // Endpoint is below the starting point
        // We just create a bridge geometry with the pillar parameters and
        // move the data.
        // SupportTreeMesher.hpp:70
        return cylinder(p.r, p.height, steps, p.endpoint());
    }

    // SupportTreeMesher.hpp:73
    indexed_triangle_set::default()
}

/// SupportTreeMesher.hpp:76-79
/// C++: `inline indexed_triangle_set get_mesh(const Pedestal &p, size_t steps)`
pub fn get_mesh_pedestal(p: &Pedestal, steps: usize) -> indexed_triangle_set {
    // SupportTreeMesher.hpp:78
    halfcone(p.height, p.r_bottom, p.r_top, &p.pos, steps)
}

/// SupportTreeMesher.hpp:81-87
/// C++: `inline indexed_triangle_set get_mesh(const Junction &j, size_t steps)`
pub fn get_mesh_junction(j: &Junction, steps: usize) -> indexed_triangle_set {
    // SupportTreeMesher.hpp:83
    let mut mesh = sphere(j.r, make_portion(0.0, PI), 2.0 * PI / steps as f64);
    // SupportTreeMesher.hpp:84
    let pos = cast_f32(&j.pos);
    // SupportTreeMesher.hpp:85
    for p in &mut mesh.vertices {
        *p += pos;
    }
    // SupportTreeMesher.hpp:86
    mesh
}

/// SupportTreeMesher.hpp:89-105
/// C++: `inline indexed_triangle_set get_mesh(const Bridge &br, size_t steps)`
pub fn get_mesh_bridge(br: &Bridge, steps: usize) -> indexed_triangle_set {
    // SupportTreeMesher.hpp:91 — using Quaternion = Eigen::Quaternion<float>;
    // SupportTreeMesher.hpp:92
    let v = br.endp - br.startp;
    // SupportTreeMesher.hpp:93
    // (Eigen `normalized()` divides by the norm unguarded; the crate
    //  `Vec3::normalized()` has an epsilon guard, so divide explicitly.)
    let dir = v / v.norm();
    // SupportTreeMesher.hpp:94
    let d = v.norm();

    // SupportTreeMesher.hpp:96 — cylinder(br.r, d, steps) with default sp = Vec3d::Zero().
    let mut mesh = cylinder(br.r, d, steps, &Vec3d::zero());

    // SupportTreeMesher.hpp:98-99
    let quater = QuaternionF::from_two_vectors(Vec3f::new(0.0, 0.0, 1.0), cast_f32(&dir));

    // SupportTreeMesher.hpp:101
    let startp = cast_f32(&br.startp);
    // SupportTreeMesher.hpp:102
    for p in &mut mesh.vertices {
        *p = quater.transform_vector(*p) + startp;
    }

    // SupportTreeMesher.hpp:104
    mesh
}

/// SupportTreeMesher.hpp:107-125
/// C++: `inline indexed_triangle_set get_mesh(const DiffBridge &br, size_t steps)`
pub fn get_mesh_diff_bridge(br: &DiffBridge, steps: usize) -> indexed_triangle_set {
    // SupportTreeMesher.hpp:109
    let h = br.get_length();
    // SupportTreeMesher.hpp:110
    let mut mesh = halfcone(h, br.r, br.end_r, &Vec3d::zero(), steps);

    // SupportTreeMesher.hpp:112 — using Quaternion = Eigen::Quaternion<float>;

    // We rotate the head to the specified direction. The head's pointing
    // side is facing upwards so this means that it would hold a support
    // point with a normal pointing straight down. This is the reason of
    // the -1 z coordinate
    // SupportTreeMesher.hpp:118-119
    let quatern = QuaternionF::from_two_vectors(Vec3f::new(0.0, 0.0, 1.0), cast_f32(&br.get_dir()));

    // SupportTreeMesher.hpp:121
    let startp = cast_f32(&br.startp);
    // SupportTreeMesher.hpp:122
    for p in &mut mesh.vertices {
        *p = quatern.transform_vector(*p) + startp;
    }

    // SupportTreeMesher.hpp:124
    mesh
}
