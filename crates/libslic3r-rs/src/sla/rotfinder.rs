//! Faithful 1:1 port of `SLA/Rotfinder.{cpp,hpp}` from BambuStudio libslic3r.
//!
//! C++ Reference:
//! - src/libslic3r/SLA/Rotfinder.hpp (73 lines)
//! - src/libslic3r/SLA/Rotfinder.cpp (477 lines)
//!
//! Fidelity notes:
//! - All scoring math reproduces the C++ float/double mixing exactly:
//!   mesh vertices are `Vec3f` (f32), `Facestats::area` is `double` (f64),
//!   `scaled<int_fast64_t>` truncates after dividing by `SCALING_FACTOR`
//!   (f32 division for the f32 overload, f64 division for the f64 overload),
//!   and `unscaled` multiplies by `SCALING_FACTOR` in f64.
//! - C++ `Transform3f` (Eigen `Transform<float,3,Affine>`) is represented as
//!   `nalgebra::Matrix4<f32>` (reusing `crate::triangle_selector::Transform3f`);
//!   `tr * Vec3f` applies the affine map `linear()*p + translation()`.
//! - The C++ functions take `const TriangleMesh &mesh` but only ever access
//!   `mesh.its` (an `indexed_triangle_set`). The crate's `TriangleMesh` is a
//!   documented divergent type without an `.its` member (see triangle_mesh.rs),
//!   so — following the convention of the other SLA ports (indexed_mesh.rs,
//!   support_tree_mesher.rs) — these functions take `&indexed_triangle_set`
//!   directly and every C++ `mesh.its.X` becomes `mesh.X`.
//!
//! BLOCKED symbols (not ported, no stubs — see the in-place comment blocks):
//! - `is_on_floor` (Rotfinder.cpp:196-202): needs `SLAPrintObjectConfig`
//!   (SLAPrint.hpp), which is not yet ported.
//! - `get_chull_rotations` (Rotfinder.cpp:205-253): needs the qhull-backed
//!   `TriangleMesh::convex_hull_3d()` and `TriangleMesh::convex_hull()`
//!   (TriangleMesh.cpp), which are not yet ported (native qhull backend).
//! - `RotfinderBoilerplate` (Rotfinder.cpp:287-325): needs the faithful
//!   `ModelObject::raw_mesh()` / `ModelInstance::get_scaling_factor()` /
//!   `ModelInstance::get_mirror()` API (Model.hpp); the crate's `ModelObject`
//!   is a documented divergent simplified type without these members.
//! - `find_best_misalignment_rotation` (Rotfinder.cpp:327-353),
//!   `find_least_supports_rotation` (Rotfinder.cpp:355-413),
//!   `find_min_z_height_rotation` (Rotfinder.cpp:432-474): each depends on
//!   `RotfinderBoilerplate` (and, for the latter two, additionally on
//!   `SLAPrintObjectConfig` / `TriangleMesh::convex_hull_3d`).

use crate::bounding_box::BoundingBoxf3;
use crate::calib::DynamicPrintConfig;
use crate::execution::{ExecutionPolicy, EX_TBB};
use crate::geometry::geometry::Transformation;
use crate::libslic3r::{unscale, SCALING_FACTOR};
use crate::normal_utils::{indexed_triangle_set, Vec3f};
use crate::triangle_selector::Transform3f;
use nalgebra::{Rotation3, Vector3, Vector4};

// libslic3r.h:59: `static constexpr double PI = 3.141592653589793238;`
// The C++ literal rounds to the same f64 bit pattern as `std::f64::consts::PI`.
use std::f64::consts::PI;

// ===========================================================================
// Rotfinder.hpp:18-42 — RotOptimizeStatusCB / RotOptimizeParams
// ===========================================================================

/// Rotfinder.hpp:18: `using RotOptimizeStatusCB = std::function<bool(int)>;`
pub type RotOptimizeStatusCB = Box<dyn Fn(i32) -> bool + Send + Sync>;

/// Rotfinder.hpp:20-42: `class RotOptimizeParams`
///
/// The C++ class holds a raw `const DynamicPrintConfig *` (the header only
/// forward-declares `class DynamicPrintConfig;`); the Rust port holds an
/// `Option<&DynamicPrintConfig>` with an explicit lifetime.
pub struct RotOptimizeParams<'a> {
    /// Rotfinder.hpp:21: `float m_accuracy = 1.;`
    m_accuracy: f32,
    /// Rotfinder.hpp:22: `const DynamicPrintConfig *m_print_config = nullptr;`
    m_print_config: Option<&'a DynamicPrintConfig>,
    /// Rotfinder.hpp:23: `RotOptimizeStatusCB m_statuscb = [](int) { return true; };`
    m_statuscb: RotOptimizeStatusCB,
}

impl<'a> Default for RotOptimizeParams<'a> {
    fn default() -> Self {
        Self {
            // Rotfinder.hpp:21
            m_accuracy: 1.0,
            // Rotfinder.hpp:22
            m_print_config: None,
            // Rotfinder.hpp:23
            m_statuscb: Box::new(|_| true),
        }
    }
}

impl<'a> RotOptimizeParams<'a> {
    /// Rotfinder.hpp:27: `RotOptimizeParams &accuracy(float a) { m_accuracy = a; return *this; }`
    pub fn accuracy(&mut self, a: f32) -> &mut Self {
        self.m_accuracy = a;
        self
    }

    /// Rotfinder.hpp:28-32: `RotOptimizeParams &print_config(const DynamicPrintConfig *c)`
    pub fn print_config(&mut self, c: Option<&'a DynamicPrintConfig>) -> &mut Self {
        // Rotfinder.hpp:30
        self.m_print_config = c;
        // Rotfinder.hpp:31
        self
    }

    /// Rotfinder.hpp:33-37: `RotOptimizeParams &statucb(RotOptimizeStatusCB cb)`
    /// (NOTE: the C++ source spells the setter `statucb`; the typo is preserved.)
    pub fn statucb(&mut self, cb: RotOptimizeStatusCB) -> &mut Self {
        // Rotfinder.hpp:35
        self.m_statuscb = cb;
        // Rotfinder.hpp:36
        self
    }

    /// Rotfinder.hpp:39: `float accuracy() const { return m_accuracy; }`
    /// (C++ overloads getter/setter by signature; Rust uses a `get_` prefix,
    /// matching the crate convention established by `optimize::StopCriteria`.)
    pub fn get_accuracy(&self) -> f32 {
        self.m_accuracy
    }

    /// Rotfinder.hpp:40: `const DynamicPrintConfig * print_config() const { return m_print_config; }`
    pub fn get_print_config(&self) -> Option<&'a DynamicPrintConfig> {
        self.m_print_config
    }

    /// Rotfinder.hpp:41: `const RotOptimizeStatusCB &statuscb() const { return m_statuscb; }`
    pub fn get_statuscb(&self) -> &RotOptimizeStatusCB {
        &self.m_statuscb
    }
}

// ===========================================================================
// Rotfinder.cpp:20-283 — anonymous namespace
// (exposed `pub(crate)` in Rust; `#[allow(dead_code)]` where the only C++
// callers are the blocked `RotfinderBoilerplate`-dependent functions)
// ===========================================================================

/// Rotfinder.cpp:22: `inline const Vec3f DOWN = {0.f, 0.f, -1.f};`
#[inline]
fn down() -> Vec3f {
    Vec3f::new(0.0, 0.0, -1.0)
}

/// Rotfinder.cpp:23: `constexpr double POINTS_PER_UNIT_AREA = 1.f;`
const POINTS_PER_UNIT_AREA: f64 = 1.0;

/// `std::thread::hardware_concurrency()` — "Returns the number of concurrent
/// threads supported... the value should be considered only a hint" and may
/// be 0 when not computable. Rotfinder.cpp:94/128/157/263.
#[inline]
fn hardware_concurrency() -> usize {
    std::thread::available_parallelism().map_or(0, |n| n.get())
}

/// Eigen `Transform3f * Vec3f` (affine point transform):
/// computes `linear() * p + translation()`, which for the stored 4x4 matrix is
/// exactly the upper three components of `m * (p.x, p.y, p.z, 1)`.
#[inline]
fn transform_point(tr: &Transform3f, p: &Vec3f) -> Vec3f {
    let v = tr * Vector4::new(p.x, p.y, p.z, 1.0f32);
    Vec3f::new(v.x, v.y, v.z)
}

/// Eigen `MatrixBase::normalized()`: returns `*this / sqrt(squaredNorm())`
/// when `squaredNorm() > 0`, otherwise returns the vector unchanged (NOT NaN —
/// this matters for degenerate, zero-area triangles).
#[inline]
fn eigen_normalized(v: &Vec3f) -> Vec3f {
    let z = v.norm_squared();
    if z > 0.0 {
        v / z.sqrt()
    } else {
        *v
    }
}

/// Point.hpp:536-542: `scaled<int_fast64_t>(const float &v)`
/// C++: `return Tout(v / Tin(SCALING_FACTOR));` with `Tin = float` — note the
/// truncating f32 division by `float(SCALING_FACTOR)` (libslic3r.h:58 = 1e-5).
#[inline]
fn scaled_int_fast64_from_f32(v: f32) -> i64 {
    (v / (SCALING_FACTOR as f32)) as i64
}

/// Point.hpp:536-542: `scaled<int_fast64_t>(const double &v)`
/// C++: `return Tout(v / Tin(SCALING_FACTOR));` with `Tin = double`.
#[inline]
fn scaled_int_fast64_from_f64(v: f64) -> i64 {
    (v / SCALING_FACTOR) as i64
}

// Get the vertices of a triangle directly in an array of 3 points
// Rotfinder.cpp:26-33
pub(crate) fn get_triangle_vertices(
    mesh: &indexed_triangle_set,
    faceidx: usize,
) -> [Vec3f; 3] {
    // Rotfinder.cpp:29
    let face = &mesh.indices[faceidx];
    // Rotfinder.cpp:30-32
    [
        mesh.vertices[face[0] as usize],
        mesh.vertices[face[1] as usize],
        mesh.vertices[face[2] as usize],
    ]
}

// Rotfinder.cpp:35-41
pub(crate) fn get_transformed_triangle(
    mesh: &indexed_triangle_set,
    tr: &Transform3f,
    faceidx: usize,
) -> [Vec3f; 3] {
    // Rotfinder.cpp:39
    let tri = get_triangle_vertices(mesh, faceidx);
    // Rotfinder.cpp:40
    [
        transform_point(tr, &tri[0]),
        transform_point(tr, &tri[1]),
        transform_point(tr, &tri[2]),
    ]
}

/// Rotfinder.cpp:43-48: `template<class T> Vec<3, T> normal(const std::array<Vec<3, T>, 3> &tri)`
/// (unused in the C++ file as well — `Facestats` recomputes the normal inline)
#[allow(dead_code)]
pub(crate) fn normal<T: nalgebra::RealField + Copy>(tri: &[Vector3<T>; 3]) -> Vector3<T> {
    // Rotfinder.cpp:45
    let u = tri[1] - tri[0];
    // Rotfinder.cpp:46
    let v = tri[2] - tri[0];
    // Rotfinder.cpp:47 — `U.cross(V).normalized()`; Eigen's normalized() keeps
    // the zero vector unchanged instead of producing NaN.
    let c = u.cross(&v);
    let z = c.norm_squared();
    if z > T::zero() {
        c / z.sqrt()
    } else {
        c
    }
}

// Rotfinder.cpp:50-59
//
// C++: `template<class T, class AccessFn> T sum_score(AccessFn &&accessfn, size_t facecount, size_t Nthreads)`
// The only instantiation in this file is `T = int_fast64_t`; `T initv = 0.`
// maps to `T::default()` (== 0 for i64).
pub(crate) fn sum_score<T, AccessFn>(accessfn: AccessFn, facecount: usize, nthreads: usize) -> T
where
    T: Clone + Send + Sync + Default + std::ops::Add<Output = T>,
    AccessFn: Fn(usize) -> T + Send + Sync,
{
    // Rotfinder.cpp:53
    let initv = T::default();
    // Rotfinder.cpp:54
    let mergefn = |a: T, b: T| a + b;
    // Rotfinder.cpp:55
    let grainsize = facecount / nthreads;
    // Rotfinder.cpp:56
    let (from, to) = (0usize, facecount);

    // Rotfinder.cpp:58
    EX_TBB.reduce(from, to, initv, mergefn, accessfn, grainsize)
}

// Get area and normal of a triangle
// Rotfinder.cpp:62-74
pub(crate) struct Facestats {
    /// Rotfinder.cpp:63
    pub normal: Vec3f,
    /// Rotfinder.cpp:64
    pub area: f64,
}

impl Facestats {
    /// Rotfinder.cpp:66-73: `explicit Facestats(const std::array<Vec3f, 3> &triangle)`
    pub fn new(triangle: &[Vec3f; 3]) -> Self {
        // Rotfinder.cpp:68
        let u = triangle[1] - triangle[0];
        // Rotfinder.cpp:69
        let v = triangle[2] - triangle[0];
        // Rotfinder.cpp:70
        let c = u.cross(&v);
        // Rotfinder.cpp:71
        let normal = eigen_normalized(&c);
        // Rotfinder.cpp:72 — `0.5 * C.norm()`: f32 norm promoted to f64, f64 multiply
        let area = 0.5f64 * (c.norm() as f64);
        Self { normal, area }
    }
}

// Try to guess the number of support points needed to support a mesh
// Rotfinder.cpp:77-98
#[allow(dead_code)]
pub(crate) fn get_misalginment_score(mesh: &indexed_triangle_set, tr: &Transform3f) -> f64 {
    // Rotfinder.cpp:79 — `if (mesh.its.vertices.empty()) return std::nan("");`
    if mesh.vertices.is_empty() {
        return f64::NAN;
    }

    // Rotfinder.cpp:81-91
    let accessfn = |fi: usize| -> i64 {
        // Rotfinder.cpp:82
        let fc = Facestats::new(&get_transformed_triangle(mesh, tr, fi));

        // Rotfinder.cpp:84-87
        // C++: `float score = fc.area * (std::abs(fc.normal.dot(UnitX))
        //                              + std::abs(fc.normal.dot(UnitY))
        //                              + std::abs(fc.normal.dot(UnitZ)));`
        // The three dot products and their sum are f32; `fc.area` (f64) times the
        // promoted sum yields f64, then narrows to f32 on assignment.
        let sum: f32 = fc.normal.dot(&Vec3f::x()).abs()
            + fc.normal.dot(&Vec3f::y()).abs()
            + fc.normal.dot(&Vec3f::z()).abs();
        let score: f32 = (fc.area * (sum as f64)) as f32;

        // We should score against the alignment with the reference planes
        // Rotfinder.cpp:90 — `return scaled<int_fast64_t>(score);`
        scaled_int_fast64_from_f32(score)
    };

    // Rotfinder.cpp:93
    let facecount = mesh.indices.len();
    // Rotfinder.cpp:94
    let nthreads = hardware_concurrency();
    // Rotfinder.cpp:95
    let s = unscale(sum_score::<i64, _>(accessfn, facecount, nthreads));

    // Rotfinder.cpp:97
    s / facecount as f64
}

// The score function for a particular face
// Rotfinder.cpp:101-115
#[inline]
pub(crate) fn get_supportedness_score(fc: &Facestats) -> f64 {
    // Simply get the angle (acos of dot product) between the face normal and
    // the DOWN vector.
    // Rotfinder.cpp:105 — `float cosphi = fc.normal.dot(DOWN);`
    let cosphi: f32 = fc.normal.dot(&down());
    // Rotfinder.cpp:106 — `float phi = 1.f - std::acos(cosphi) / float(PI);`
    let phi: f32 = 1.0f32 - cosphi.acos() / (PI as f32);

    // Make the huge slopes more significant than the smaller slopes
    // Rotfinder.cpp:109
    let phi = phi * phi * phi;

    // Multiply with the square root of face area of the current face,
    // the area is less important as it grows.
    // This makes many smaller overhangs a bigger impact.
    // Rotfinder.cpp:114 — `std::sqrt(fc.area) * POINTS_PER_UNIT_AREA * phi`
    fc.area.sqrt() * POINTS_PER_UNIT_AREA * (phi as f64)
}

// Try to guess the number of support points needed to support a mesh
// Rotfinder.cpp:118-132
//
// C++ overloads `get_supportedness_score` on (const TriangleMesh &, const Transform3f &);
// Rust cannot overload, hence the `_mesh` suffix.
#[allow(dead_code)]
pub(crate) fn get_supportedness_score_mesh(
    mesh: &indexed_triangle_set,
    tr: &Transform3f,
) -> f64 {
    // Rotfinder.cpp:120
    if mesh.vertices.is_empty() {
        return f64::NAN;
    }

    // Rotfinder.cpp:122-125
    let accessfn = |fi: usize| -> i64 {
        // Rotfinder.cpp:123
        let fc = Facestats::new(&get_transformed_triangle(mesh, tr, fi));
        // Rotfinder.cpp:124 — `scaled<int_fast64_t>` over a double here
        scaled_int_fast64_from_f64(get_supportedness_score(&fc))
    };

    // Rotfinder.cpp:127
    let facecount = mesh.indices.len();
    // Rotfinder.cpp:128
    let nthreads = hardware_concurrency();
    // Rotfinder.cpp:129
    let s = unscale(sum_score::<i64, _>(accessfn, facecount, nthreads));

    // Rotfinder.cpp:131
    s / facecount as f64
}

// Find transformed mesh ground level without copy and with parallel reduce.
// Rotfinder.cpp:135-150
pub(crate) fn find_ground_level(
    mesh: &indexed_triangle_set,
    tr: &Transform3f,
    threads: usize,
) -> f32 {
    // Rotfinder.cpp:139
    let vsize = mesh.vertices.len();

    // Rotfinder.cpp:141 — `std::min(a, b)`: returns b if b < a, else a
    let minfn = |a: f32, b: f32| if b < a { b } else { a };

    // Rotfinder.cpp:143-145
    let accessfn = |vi: usize| transform_point(tr, &mesh.vertices[vi]).z;

    // Rotfinder.cpp:147
    let zmin = f32::MAX;
    // Rotfinder.cpp:148
    let granularity = vsize / threads;
    // Rotfinder.cpp:149
    EX_TBB.reduce(0usize, vsize, zmin, minfn, accessfn, granularity)
}

// Rotfinder.cpp:152-176
#[allow(dead_code)]
pub(crate) fn get_supportedness_onfloor_score(
    mesh: &indexed_triangle_set,
    tr: &Transform3f,
) -> f32 {
    // Rotfinder.cpp:155
    if mesh.vertices.is_empty() {
        return f32::NAN;
    }

    // Rotfinder.cpp:157
    let nthreads = hardware_concurrency();

    // Rotfinder.cpp:159
    let zmin = find_ground_level(mesh, tr, nthreads);
    // Rotfinder.cpp:160 — Set up a slight tolerance from z level
    let zlvl = zmin + 0.1f32;

    // Rotfinder.cpp:162-170
    let accessfn = |fi: usize| -> i64 {
        // Rotfinder.cpp:163
        let tri = get_transformed_triangle(mesh, tr, fi);
        // Rotfinder.cpp:164
        let fc = Facestats::new(&tri);

        // Rotfinder.cpp:166-169
        let s: f64 = if tri[0].z <= zlvl && tri[1].z <= zlvl && tri[2].z <= zlvl {
            // Rotfinder.cpp:167 — `-2 * fc.area * POINTS_PER_UNIT_AREA`
            -2.0f64 * fc.area * POINTS_PER_UNIT_AREA
        } else {
            // Rotfinder.cpp:169
            get_supportedness_score(&fc)
        };

        // The C++ lambda returns `double`, but `sum_score<int_fast64_t>`'s
        // mergefn takes `int_fast64_t` parameters, so each per-face score is
        // implicitly converted (truncated toward zero) inside
        // `execution::reduce` (ExecutionTBB.hpp:58: `acc = mergefn(acc, access(i))`).
        // NOTE: unlike the other two scoring functions there is NO
        // `scaled<int_fast64_t>` here — this truncation of small raw scores is
        // upstream behavior and is reproduced faithfully.
        s as i64
    };

    // Rotfinder.cpp:172
    let facecount = mesh.indices.len();
    // Rotfinder.cpp:173
    let s = unscale(sum_score::<i64, _>(accessfn, facecount, nthreads));

    // Rotfinder.cpp:175 — double expression returned as float
    (s / facecount as f64) as f32
}

/// Rotfinder.cpp:178: `using XYRotation = std::array<double, 2>;`
pub type XYRotation = [f64; 2];

// prepare the rotation transformation
// Rotfinder.cpp:181-188
#[allow(dead_code)]
pub(crate) fn to_transform3f(rot: &XYRotation) -> Transform3f {
    // Rotfinder.cpp:183
    let mut rt = Transform3f::identity();
    // Rotfinder.cpp:184 — `rt.rotate(Eigen::AngleAxisf(float(rot[1]), Vec3f::UnitY()));`
    // Eigen's Transform::rotate applies the rotation on the right: rt = rt * R.
    rt *= Rotation3::from_axis_angle(&Vector3::y_axis(), rot[1] as f32).to_homogeneous();
    // Rotfinder.cpp:185 — `rt.rotate(Eigen::AngleAxisf(float(rot[0]), Vec3f::UnitX()));`
    rt *= Rotation3::from_axis_angle(&Vector3::x_axis(), rot[0] as f32).to_homogeneous();

    // Rotfinder.cpp:187
    rt
}

// Rotfinder.cpp:190-194
#[allow(dead_code)]
pub(crate) fn from_transform3f(tr: &Transform3f) -> XYRotation {
    // Rotfinder.cpp:192 — `Vec3d rot3 = Geometry::Transformation{tr.cast<double>()}.get_rotation();`
    let rot3 = Transformation::from_transform(tr.cast::<f64>()).get_rotation();
    // Rotfinder.cpp:193
    [rot3.x, rot3.y]
}

// Rotfinder.cpp:196-202 — BLOCKED, not ported (no stub):
//
//     inline bool is_on_floor(const SLAPrintObjectConfig &cfg)
//     {
//         auto opt_elevation = cfg.support_object_elevation.getFloat();
//         auto opt_padaround = cfg.pad_around_object.getBool();
//
//         return opt_elevation < EPSILON || opt_padaround;
//     }
//
// Reason: `SLAPrintObjectConfig` (SLAPrint.hpp) is not yet ported; faithfully
// porting it requires the reflective `ConfigOption` machinery from
// PrintConfig.hpp threaded through the SLA print pipeline.

// collect the rotations for each face of the convex hull
// Rotfinder.cpp:205-253 — BLOCKED, not ported (no stub):
//
//     std::vector<XYRotation> get_chull_rotations(const TriangleMesh &mesh, size_t max_count)
//
// Reason: depends on the qhull-backed `TriangleMesh::convex_hull_3d()` and on
// `TriangleMesh::convex_hull()` (TriangleMesh.cpp), both of which are listed
// as blocked in triangle_mesh.rs (native qhull backend; the crate-level
// `geometry::convex_hull_3d` is only the 2D XY-projected hull from
// Geometry/ConvexHull.cpp, not a 3D hull mesh).

// Find the best score from a set of function inputs. Evaluate for every point.
// Rotfinder.cpp:256-281
//
// C++: `template<size_t N, class Fn, class It, class StopCond>
//       std::array<double, N> find_min_score(Fn &&fn, It from, It to, StopCond &&stopfn)`
// The C++ iterator pair (from, to) over `XYRotation`s maps to a slice.
#[allow(dead_code)]
pub(crate) fn find_min_score<const N: usize, F, S>(
    fn_: F,
    inputs: &[[f64; N]],
    stopfn: S,
) -> [f64; N]
where
    F: Fn(&[f64; N]) -> f64 + Send + Sync,
    S: Fn() -> bool + Send + Sync,
{
    // Rotfinder.cpp:259 — `std::array<double, N> ret = {};`
    let mut ret = [0.0f64; N];

    // Rotfinder.cpp:261
    let score = f64::MAX;

    // Rotfinder.cpp:263
    let nthreads = hardware_concurrency();
    // Rotfinder.cpp:264
    let dist = inputs.len();
    // Rotfinder.cpp:265 — `std::vector<double> scores(dist, score);`
    // C++ writes `scores[i]` concurrently from TBB worker threads at disjoint
    // indices; Rust expresses the same disjoint writes with relaxed atomic
    // stores of the f64 bit patterns.
    let scores: Vec<std::sync::atomic::AtomicU64> = (0..dist)
        .map(|_| std::sync::atomic::AtomicU64::new(score.to_bits()))
        .collect();

    // Rotfinder.cpp:267-273
    EX_TBB.for_each(
        0usize,
        dist,
        |i| {
            // Rotfinder.cpp:269
            if stopfn() {
                return;
            }

            // Rotfinder.cpp:271 — `scores[i] = fn(*(from + i));`
            scores[i].store(
                fn_(&inputs[i]).to_bits(),
                std::sync::atomic::Ordering::Relaxed,
            );
        },
        // Rotfinder.cpp:273
        dist / nthreads,
    );

    // Rotfinder.cpp:275 — `auto it = std::min_element(scores.begin(), scores.end());`
    // std::min_element keeps the first element for which no later element
    // compares strictly smaller (operator<).
    let vals: Vec<f64> = scores
        .iter()
        .map(|s| f64::from_bits(s.load(std::sync::atomic::Ordering::Relaxed)))
        .collect();
    if !vals.is_empty() {
        let mut min_i = 0usize;
        for i in 1..vals.len() {
            if vals[i] < vals[min_i] {
                min_i = i;
            }
        }
        // Rotfinder.cpp:277-278 — `ret = *(from + std::distance(scores.begin(), it));`
        ret = inputs[min_i];
    }

    // Rotfinder.cpp:280
    ret
}

// ===========================================================================
// Rotfinder.cpp:287-325 — BLOCKED, not ported (no stub):
//
//     template<unsigned MAX_ITER>
//     struct RotfinderBoilerplate {
//         static constexpr unsigned MAX_TRIES = MAX_ITER;
//
//         int status = 0;
//         TriangleMesh mesh;
//         unsigned max_tries;
//         const RotOptimizeParams &params;
//
//         static TriangleMesh get_mesh_to_rotate(const ModelObject &mo) { ... }
//         RotfinderBoilerplate(const ModelObject &mo, const RotOptimizeParams &p) ...
//         void statusfn() { params.statuscb()(++status * 100.0 / max_tries); }
//         bool stopcond() { return ! params.statuscb()(-1); }
//     };
//
// Reason: `get_mesh_to_rotate` (Rotfinder.cpp:298-313) requires the faithful
// `ModelObject::raw_mesh()`, `ModelInstance::get_scaling_factor()`,
// `ModelInstance::get_mirror()` (Model.hpp) and `TriangleMesh::transform`
// over the C++-layout mesh; the crate's `ModelObject`/`Instance` are documented
// divergent simplified types without these members (same blocker is recorded by
// slicing_adaptive.rs and sla/reproject_points_on_mesh.rs).
// ===========================================================================

// Rotfinder.cpp:327-353 — BLOCKED, not ported (no stub):
//
//     Vec2d find_best_misalignment_rotation(const ModelObject &mo,
//                                           const RotOptimizeParams &params)
//
// Rotfinder.hpp:44-59 documents it as: find the best rotation for SLA upside
// down printing (brute-force `opt::Optimizer<opt::AlgBruteForce>` over
// [-PI, PI]^2 maximizing `get_misalginment_score`).
// Reason: requires `RotfinderBoilerplate` (blocked above). The optimizer
// (`optimize::BruteForceOptimizer`) and the objective
// (`get_misalginment_score` + `to_transform3f`) are already ported.

// Rotfinder.cpp:355-413 — BLOCKED, not ported (no stub):
//
//     Vec2d find_least_supports_rotation(const ModelObject &mo,
//                                        const RotOptimizeParams &params)
//
// Reason: requires `RotfinderBoilerplate` (blocked above), `SLAPrintObjectConfig`
// + `is_on_floor` (blocked above) and `get_chull_rotations` (blocked above).
// The non-floor branch's machinery (`BruteForceOptimizer`,
// `get_supportedness_score_mesh`, `find_min_score`,
// `get_supportedness_onfloor_score`) is already ported.

// Rotfinder.cpp:415-430
#[inline]
pub fn bounding_box_with_tr(its: &indexed_triangle_set, tr: &Transform3f) -> BoundingBoxf3 {
    // Rotfinder.cpp:418-419 — `return {};`
    if its.vertices.is_empty() {
        return BoundingBoxf3::new();
    }

    // Rotfinder.cpp:421
    let front = transform_point(tr, &its.vertices[0]);
    let mut bmin = front;
    let mut bmax = front;

    // Rotfinder.cpp:423-427
    for p in &its.vertices {
        // Rotfinder.cpp:424
        let pp = transform_point(tr, p);
        // Rotfinder.cpp:425 — `bmin = pp.cwiseMin(bmin);`
        bmin = pp.inf(&bmin);
        // Rotfinder.cpp:426 — `bmax = pp.cwiseMax(bmax);`
        bmax = pp.sup(&bmax);
    }

    // Rotfinder.cpp:429 — `{bmin.cast<double>(), bmax.cast<double>()}`
    // (BoundingBox3Base two-point constructor, which computes `defined`)
    BoundingBoxf3::new_from_points(
        crate::geometry::Vec3d::new(bmin.x as f64, bmin.y as f64, bmin.z as f64),
        crate::geometry::Vec3d::new(bmax.x as f64, bmax.y as f64, bmax.z as f64),
    )
}

// Rotfinder.cpp:432-474 — BLOCKED, not ported (no stub):
//
//     Vec2d find_min_z_height_rotation(const ModelObject &mo,
//                                      const RotOptimizeParams &params)
//
// Reason: requires `RotfinderBoilerplate` (blocked above) and the qhull-backed
// `TriangleMesh::convex_hull_3d()` (blocked, see `get_chull_rotations`), plus
// `Eigen::Quaternionf{}.FromTwoVectors(fc.normal, DOWN)` over the hull faces.
// Its helpers `bounding_box_with_tr`, `from_transform3f`, `to_transform3f`
// and `find_min_score` are already ported.

#[cfg(test)]
mod tests {
    use super::*;

    fn unit_triangle_its() -> indexed_triangle_set {
        indexed_triangle_set {
            vertices: vec![
                Vec3f::new(0.0, 0.0, 0.0),
                Vec3f::new(1.0, 0.0, 0.0),
                Vec3f::new(0.0, 1.0, 0.0),
            ],
            indices: vec![Vector3::new(0, 1, 2)],
        }
    }

    #[test]
    fn test_facestats_unit_triangle() {
        let tri = [
            Vec3f::new(0.0, 0.0, 0.0),
            Vec3f::new(1.0, 0.0, 0.0),
            Vec3f::new(0.0, 1.0, 0.0),
        ];
        let fc = Facestats::new(&tri);
        assert_eq!(fc.area, 0.5);
        assert_eq!(fc.normal, Vec3f::new(0.0, 0.0, 1.0));
    }

    #[test]
    fn test_facestats_degenerate_triangle_keeps_zero_normal() {
        // Eigen normalized() returns the zero vector unchanged.
        let tri = [
            Vec3f::new(0.0, 0.0, 0.0),
            Vec3f::new(1.0, 0.0, 0.0),
            Vec3f::new(2.0, 0.0, 0.0),
        ];
        let fc = Facestats::new(&tri);
        assert_eq!(fc.area, 0.0);
        assert_eq!(fc.normal, Vec3f::new(0.0, 0.0, 0.0));
    }

    #[test]
    fn test_get_misalginment_score_axis_aligned() {
        let its = unit_triangle_its();
        let tr = Transform3f::identity();
        // score per face = area * (|nx|+|ny|+|nz|) = 0.5 * 1 = 0.5
        // scaled -> 50000, unscaled -> 0.5, / facecount=1 -> 0.5
        let s = get_misalginment_score(&its, &tr);
        assert!((s - 0.5).abs() < 1e-9, "s = {s}");
    }

    #[test]
    fn test_get_misalginment_score_empty_mesh_nan() {
        let its = indexed_triangle_set::default();
        assert!(get_misalginment_score(&its, &Transform3f::identity()).is_nan());
    }

    #[test]
    fn test_get_supportedness_score_down_facing() {
        // Normal pointing DOWN: cosphi = 1, acos = 0, phi = 1 -> sqrt(area).
        let fc = Facestats {
            normal: Vec3f::new(0.0, 0.0, -1.0),
            area: 4.0,
        };
        let s = get_supportedness_score(&fc);
        assert!((s - 2.0).abs() < 1e-6, "s = {s}");
    }

    #[test]
    fn test_find_ground_level_identity() {
        let its = unit_triangle_its();
        let z = find_ground_level(&its, &Transform3f::identity(), 4);
        assert_eq!(z, 0.0);
    }

    #[test]
    fn test_to_from_transform3f_roundtrip() {
        let rot: XYRotation = [0.3, 0.5];
        let tr = to_transform3f(&rot);
        let back = from_transform3f(&tr);
        assert!((back[0] - 0.3).abs() < 1e-5, "x = {}", back[0]);
        assert!((back[1] - 0.5).abs() < 1e-5, "y = {}", back[1]);
    }

    #[test]
    fn test_bounding_box_with_tr_identity() {
        let its = unit_triangle_its();
        let bb = bounding_box_with_tr(&its, &Transform3f::identity());
        assert_eq!(bb.min.x, 0.0);
        assert_eq!(bb.min.y, 0.0);
        assert_eq!(bb.max.x, 1.0);
        assert_eq!(bb.max.y, 1.0);
        // flat in z -> `defined` is false per BoundingBox.hpp:110-112 semantics
        assert!(!bb.defined);
    }

    #[test]
    fn test_bounding_box_with_tr_empty() {
        let its = indexed_triangle_set::default();
        let bb = bounding_box_with_tr(&its, &Transform3f::identity());
        assert!(!bb.defined);
    }

    #[test]
    fn test_find_min_score_picks_minimum() {
        let inputs: Vec<[f64; 2]> = vec![[3.0, 0.0], [1.0, 0.0], [2.0, 0.0]];
        let best = find_min_score(|x: &[f64; 2]| x[0], &inputs, || false);
        assert_eq!(best, [1.0, 0.0]);
    }

    #[test]
    fn test_find_min_score_empty_inputs() {
        let inputs: Vec<[f64; 2]> = Vec::new();
        let best = find_min_score(|x: &[f64; 2]| x[0], &inputs, || false);
        assert_eq!(best, [0.0, 0.0]);
    }

    #[test]
    fn test_sum_score_adds() {
        let s: i64 = sum_score(|i: usize| i as i64, 5, 4);
        assert_eq!(s, 10);
    }

    #[test]
    fn test_rot_optimize_params_defaults() {
        let p = RotOptimizeParams::default();
        assert_eq!(p.get_accuracy(), 1.0);
        assert!(p.get_print_config().is_none());
        assert!((p.get_statuscb())(50));
    }
}
