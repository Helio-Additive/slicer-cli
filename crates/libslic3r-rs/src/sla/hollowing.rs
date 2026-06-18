//! Faithful 1:1 port of `SLA/Hollowing.{hpp,cpp}` from BambuStudio libslic3r.
//!
//! C++ Reference:
//! - src/libslic3r/SLA/Hollowing.hpp (109 lines)
//! - src/libslic3r/SLA/Hollowing.cpp (563 lines)
//!
//! ## Native-dependency status (PARTIAL port)
//!
//! The interior-generation pipeline is built on the **native** OpenVDB level-set
//! grid (`openvdb::FloatGrid`) through `OpenVDBUtils.hpp` (`mesh_to_grid`,
//! `redistance_grid`, `grid_to_mesh`) and the grid's `ConstAccessor` voxel
//! lookups. OpenVDB is a heavyweight native C++ dependency (TBB, Boost, Blosc)
//! with no pure-Rust port; per the porting rules we must NOT add a native /
//! dylib dependency (not wasm-safe). Mirroring the crate's established pattern
//! (`src/open_vdb_utils.rs`, `src/csg_mesh/voxelize_csg_mesh.rs`), the grid is
//! modelled by the existing `VoxelGrid` placeholder and every call that crosses
//! into native OpenVDB returns an explicit `Err` instead of fake data, while
//! ALL surrounding control flow is ported exactly:
//!   - `generate_interior_verbose` (Hollowing.cpp:60-109) — errs at `mesh_to_grid`
//!   - `get_distance_raw` (Hollowing.cpp:325-336) — errs at the accessor lookup
//!
//! Everything else (DrainHole geometry, drain-hole cutting, triangle division /
//! trimming logic, mesh post-processing) is fully ported.
//!
//! Fidelity notes:
//! - C++ functions taking `TriangleMesh &` only ever access `mesh.its`, `.merge`
//!   (== `its_merge`), `.empty()` (== `its.indices.empty()`) and
//!   `.bounding_box()`; following the crate-wide SLA convention (see
//!   `sla/rotfinder.rs`) they take `indexed_triangle_set` here.
//! - Eigen `Hyperplane<float,3>` / `ParametrizedLine<float,3>` /
//!   `Quaternionf::setFromTwoVectors` have no nalgebra equivalents with
//!   identical numerics; exact local analogs are implemented below (same
//!   formulas as the Eigen 3.4 headers used by BambuStudio).
//! - `coordf_t`/`double` -> f64, `float` -> f32, `long` -> i64 per convention.

use std::cell::Cell;

use crate::clipper_utils::difference;
use crate::csg_mesh::voxelize_csg_mesh::VoxelGrid;
use crate::geometry::ExPolygons;
use crate::geometry::Point3F;
use crate::geometry::Vec3d as GeoVec3d;
use crate::libslic3r::EPSILON;
use crate::normal_utils::indexed_triangle_set;
use crate::quadric_edge_collapse::its_quadric_edge_collapse;
use crate::sla::indexed_mesh::{hit_result, Vec3d};
use crate::sla::job_controller::JobController;
use crate::sla::support_tree_mesher::cylinder;
use crate::triangle_mesh::{
    its_compactify_vertices, its_merge, its_merge_vertices, Triangle, TriangleMesh, Vec3f, Vec3i,
};
use crate::triangle_mesh_slicer::{slice_mesh_ex_its, MeshSlicingParamsEx};
use crate::bounding_box::BoundingBoxf3;
use crate::{Error, Result};

// Hollowing.cpp:18-20
// //! macro used to mark string used at localization, return same string
// #define L(s) Slic3r::I18N::translate(s)
// (the translation is an identity passthrough here; plain &str is used)

// Hollowing.cpp:22-23  namespace Slic3r { namespace sla {

// ============================================================================
// Header section (Hollowing.hpp)
// ============================================================================

/// Hollowing.hpp:12-18
#[derive(Debug, Clone, PartialEq)]
pub struct HollowingConfig {
    // Hollowing.hpp:14  double min_thickness    = 2.;
    pub min_thickness: f64,
    // Hollowing.hpp:15  double quality          = 0.5;
    pub quality: f64,
    // Hollowing.hpp:16  double closing_distance = 0.5;
    pub closing_distance: f64,
    // Hollowing.hpp:17  bool enabled = true;
    pub enabled: bool,
}

impl Default for HollowingConfig {
    fn default() -> Self {
        Self {
            min_thickness: 2.0,
            quality: 0.5,
            closing_distance: 0.5,
            enabled: true,
        }
    }
}

/// Hollowing.hpp:20  enum HollowingFlags { hfRemoveInsideTriangles = 0x1 };
#[allow(non_camel_case_types)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HollowingFlags {
    hfRemoveInsideTriangles = 0x1,
}

// Hollowing.hpp:22-26
// All data related to a generated mesh interior. Includes the 3D grid and mesh
// and various metadata. No need to manipulate from outside.
//
// Hollowing.hpp:25  struct InteriorDeleter { void operator()(Interior *p); };
// Hollowing.hpp:26  using  InteriorPtr = std::unique_ptr<Interior, InteriorDeleter>;
//
// `InteriorDeleter::operator()(Interior *p) { delete p; }` (Hollowing.cpp:45-48)
// exists in C++ only because `Interior` is an incomplete type in the header;
// in Rust `Box<Interior>`'s `Drop` is exactly that deleter. The `unique_ptr`'s
// nullability is modelled with `Option`.
pub type InteriorPtr = Option<Box<Interior>>;

// (DrainHole is declared in the header, Hollowing.hpp:31-67; its member
// functions are defined in the .cpp and ported in source order below.)

/// Hollowing.hpp:31-67
#[derive(Debug, Clone)]
pub struct DrainHole {
    // Hollowing.hpp:33  Vec3f pos;
    pub pos: Vec3f,
    // Hollowing.hpp:34  Vec3f normal;
    pub normal: Vec3f,
    // Hollowing.hpp:35  float radius;
    pub radius: f32,
    // Hollowing.hpp:36  float height;
    pub height: f32,
    // Hollowing.hpp:37  bool  failed = false;
    pub failed: bool,
}

impl Default for DrainHole {
    /// Hollowing.hpp:39-41
    /// C++: `DrainHole(): pos(Vec3f::Zero()), normal(Vec3f::UnitZ()), radius(5.f), height(10.f) {}`
    fn default() -> Self {
        Self {
            pos: Vec3f::zeros(),
            normal: Vec3f::new(0.0, 0.0, 1.0),
            radius: 5.0,
            height: 10.0,
            failed: false,
        }
    }
}

impl DrainHole {
    /// Hollowing.hpp:43-45
    /// C++: `DrainHole(Vec3f p, Vec3f n, float r, float h, bool fl = false)`
    /// (the copy constructor, Hollowing.hpp:47-48, is `#[derive(Clone)]`)
    pub fn new(p: Vec3f, n: Vec3f, r: f32, h: f32, fl: bool) -> Self {
        Self {
            pos: p,
            normal: n,
            radius: r,
            height: h,
            failed: fl,
        }
    }

    // Hollowing.hpp:61-64  template<class Archive> inline void serialize(Archive &ar)
    // { ar(pos, normal, radius, height, failed); }
    // NOT ported: cereal archive serialization has no counterpart in this crate.

    /// Hollowing.hpp:66  static constexpr size_t steps = 32;
    pub const STEPS: usize = 32;
}

// Hollowing.hpp:50  bool operator==(const DrainHole &sp) const;  (body Hollowing.cpp:162-167)
impl PartialEq for DrainHole {
    fn eq(&self, sp: &Self) -> bool {
        // Hollowing.cpp:164-166
        self.pos == sp.pos
            && self.normal == sp.normal
            && is_approx_f32(self.radius, sp.radius)
            && is_approx_f32(self.height, sp.height)
    }
}
// Hollowing.hpp:52  bool operator!=(const DrainHole &sp) const { return !(sp == (*this)); }
// (provided by `PartialEq`)

/// Hollowing.hpp:69  using DrainHoles = std::vector<DrainHole>;
pub type DrainHoles = Vec<DrainHole>;

/// Hollowing.hpp:71  constexpr float HoleStickOutLength = 1.f;
pub const HOLE_STICK_OUT_LENGTH: f32 = 1.0;

/// Hollowing.hpp:100-104
/// C++: `inline void swap_normals(indexed_triangle_set &its)`
pub fn swap_normals(its: &mut indexed_triangle_set) {
    // Hollowing.hpp:102-103  for (auto &face : its.indices) std::swap(face(0), face(2));
    for face in &mut its.indices {
        let tmp = face[0];
        face[0] = face[2];
        face[2] = tmp;
    }
}

// ============================================================================
// Eigen analogs (float precision), used by the DrainHole member functions.
// Same formulas as the Eigen 3.4 headers compiled into BambuStudio.
// ============================================================================

/// libslic3r.h:287-291  `is_approx(Number value, Number test_value, Number precision = EPSILON)`
/// C++: `return std::fabs(double(value) - double(test_value)) < double(precision);`
/// Number = float here, so the default `precision` is the double EPSILON (1e-4)
/// narrowed to f32, then widened back to f64 for the comparison.
#[inline]
fn is_approx_f32(value: f32, test_value: f32) -> bool {
    (value as f64 - test_value as f64).abs() < (EPSILON as f32) as f64
}

// Eigen/src/Geometry/Hyperplane.h — `Eigen::Hyperplane<float, 3>`.
struct Hyperplane3f {
    normal: Vec3f,
    offset: f32,
}

impl Hyperplane3f {
    // Hyperplane(const VectorType& n, const VectorType& e)
    //   : normal = n, offset = -n.dot(e)
    fn new(n: Vec3f, e: Vec3f) -> Self {
        Self {
            normal: n,
            offset: -n.dot(&e),
        }
    }

    // Scalar signedDistance(const VectorType& p) const { return normal().dot(p) + offset(); }
    fn signed_distance(&self, p: &Vec3f) -> f32 {
        self.normal.dot(p) + self.offset
    }

    // VectorType projection(const VectorType& p) const { return p - signedDistance(p) * normal(); }
    fn projection(&self, p: &Vec3f) -> Vec3f {
        p - self.signed_distance(p) * self.normal
    }
}

// Eigen/src/Geometry/ParametrizedLine.h — `Eigen::ParametrizedLine<float, 3>`.
struct ParametrizedLine3f {
    origin: Vec3f,
    direction: Vec3f,
}

impl ParametrizedLine3f {
    // ParametrizedLine(const VectorType& origin, const VectorType& direction)
    fn new(origin: Vec3f, direction: Vec3f) -> Self {
        Self { origin, direction }
    }

    // RealScalar squaredDistance(const VectorType& p) const
    // { VectorType diff = p - origin(); return (diff - diff.dot(direction()) * direction()).squaredNorm(); }
    fn squared_distance(&self, p: &Vec3f) -> f32 {
        let diff = p - self.origin;
        (diff - diff.dot(&self.direction) * self.direction).norm_squared()
    }

    // VectorType pointAt(const Scalar& t) const { return origin() + t * direction(); }
    fn point_at(&self, t: f32) -> Vec3f {
        self.origin + t * self.direction
    }

    // VectorType projection(const VectorType& p) const
    // { return origin() + direction().dot(p - origin()) * direction(); }
    fn projection(&self, p: &Vec3f) -> Vec3f {
        self.origin + self.direction.dot(&(p - self.origin)) * self.direction
    }

    // Scalar intersectionParameter(const Hyperplane& hyperplane) const
    // { return -(hyperplane.offset() + hyperplane.normal().dot(origin()))
    //          / hyperplane.normal().dot(direction()); }
    fn intersection_parameter(&self, hyperplane: &Hyperplane3f) -> f32 {
        -(hyperplane.offset + hyperplane.normal.dot(&self.origin))
            / hyperplane.normal.dot(&self.direction)
    }

    // VectorType intersectionPoint(const Hyperplane& hyperplane) const
    // { return pointAt(intersectionParameter(hyperplane)); }
    fn intersection_point(&self, hyperplane: &Hyperplane3f) -> Vec3f {
        self.point_at(self.intersection_parameter(hyperplane))
    }
}

// Eigen/src/Geometry/Quaternion.h — minimal `Eigen::Quaternionf` (w + vec parts).
struct Quaternionf {
    w: f32,
    vec: Vec3f,
}

impl Quaternionf {
    // QuaternionBase::setFromTwoVectors(const MatrixBase& a, const MatrixBase& b)
    // (Eigen/src/Geometry/Quaternion.h) — returns the rotation taking `a` to `b`.
    fn set_from_two_vectors(a: &Vec3f, b: &Vec3f) -> Self {
        // Vector3 v0 = a.normalized(); Vector3 v1 = b.normalized();
        let v0 = a.normalize();
        let v1 = b.normalize();
        // Scalar c = v1.dot(v0);
        let mut c = v1.dot(&v0);

        // if dot == -1, vectors are nearly opposites
        // (NumTraits<float>::dummy_precision() == 1e-5f)
        if c < -1.0 + 1e-5 {
            // c = numext::maxi(c, Scalar(-1));
            c = c.max(-1.0);
            // FIDELITY-NOTE(eigen-svd): Eigen solves a 2x3 JacobiSVD on
            // [v0^T; v1^T] and takes matrixV().col(2) as the rotation axis (a
            // unit vector orthogonal to both v0 and v1). We instead use Eigen's
            // own `unitOrthogonal()` algorithm (Eigen/src/Geometry/OrthoMethods.h)
            // for the axis. For (nearly) opposite vectors the rotation is ~180
            // degrees about *some* axis orthogonal to v0; the two algorithms may
            // pick different orthogonal axes within the 1e-5 dummy_precision
            // window. The sole caller (`DrainHole::to_mesh`) rotates a
            // rotationally-symmetric cylinder, so any such axis yields a
            // geometrically equivalent mesh. A byte-faithful match would require
            // porting Eigen's 2x3 JacobiSVD (foundational, out of scope here).
            let axis = unit_orthogonal_f32(&v0);
            // Scalar w2 = (Scalar(1)+c)*Scalar(0.5);
            let w2 = (1.0 + c) * 0.5;
            // this->w() = sqrt(w2); this->vec() = axis * sqrt(Scalar(1) - w2);
            return Self {
                w: w2.sqrt(),
                vec: axis * (1.0 - w2).sqrt(),
            };
        }
        // Vector3 axis = v0.cross(v1);
        let axis = v0.cross(&v1);
        // Scalar s = sqrt((Scalar(1)+c)*Scalar(2));
        let s = ((1.0 + c) * 2.0).sqrt();
        // Scalar invs = Scalar(1)/s;
        let invs = 1.0 / s;
        // this->vec() = axis * invs; this->w() = s * Scalar(0.5);
        Self {
            w: s * 0.5,
            vec: axis * invs,
        }
    }

    // QuaternionBase::_transformVector(const Vector3& v) — `q * p`:
    // Vector3 uv = this->vec().cross(v); uv += uv;
    // return v + this->w() * uv + this->vec().cross(uv);
    fn transform_vector(&self, v: &Vec3f) -> Vec3f {
        let mut uv = self.vec.cross(v);
        uv += uv;
        v + self.w * uv + self.vec.cross(&uv)
    }
}

// Eigen/src/Geometry/OrthoMethods.h — `unitOrthogonal()` for a 3-vector:
// if((!isMuchSmallerThan(src.x(), src.z())) || (!isMuchSmallerThan(src.y(), src.z())))
// { invnm = 1/sqrt(x^2+y^2); perp = (-y*invnm, x*invnm, 0); }
// else { invnm = 1/sqrt(y^2+z^2); perp = (0, -z*invnm, y*invnm); }
fn unit_orthogonal_f32(src: &Vec3f) -> Vec3f {
    // isMuchSmallerThan(x, y, prec) == |x| <= |y| * prec, prec = 1e-5f.
    let prec: f32 = 1e-5;
    let much_smaller = |x: f32, y: f32| x.abs() <= y.abs() * prec;
    if !much_smaller(src.x, src.z) || !much_smaller(src.y, src.z) {
        let invnm = 1.0 / (src.x * src.x + src.y * src.y).sqrt();
        Vec3f::new(-src.y * invnm, src.x * invnm, 0.0)
    } else {
        let invnm = 1.0 / (src.y * src.y + src.z * src.z).sqrt();
        Vec3f::new(0.0, -src.z * invnm, src.y * invnm)
    }
}

// ============================================================================
// Hollowing.cpp
// ============================================================================

/// Hollowing.cpp:25-43  struct Interior
pub struct Interior {
    // Hollowing.cpp:26  indexed_triangle_set mesh;
    pub(crate) mesh: indexed_triangle_set,
    // Hollowing.cpp:27  openvdb::FloatGrid::Ptr gridptr;
    // BLOCKED native: the OpenVDB FloatGrid is modelled by the crate's
    // `VoxelGrid` placeholder (see `csg_mesh::voxelize_csg_mesh`); the `Ptr`'s
    // nullability by `Option`.
    pub(crate) gridptr: Option<VoxelGrid>,
    // Hollowing.cpp:28  mutable std::optional<openvdb::FloatGrid::ConstAccessor> accessor;
    // BLOCKED native: the accessor (a voxel-lookup cache into the grid) cannot
    // be modelled; only its `optional` engaged/empty state is kept so the
    // `reset_accessor` control flow stays exact. (`mutable` -> `Cell`.)
    accessor: Cell<Option<()>>,

    // Hollowing.cpp:30  double closing_distance = 0.;
    pub closing_distance: f64,
    // Hollowing.cpp:31  double thickness = 0.;
    pub thickness: f64,
    // Hollowing.cpp:32  double voxel_scale = 1.;
    pub voxel_scale: f64,
    // Hollowing.cpp:33  double nb_in = 3.;  // narrow band width inwards
    pub nb_in: f64,
    // Hollowing.cpp:34  double nb_out = 3.; // narrow band width outwards
    pub nb_out: f64,
    // Full narrow band is the sum of the two above values. (Hollowing.cpp:35)
}

impl Default for Interior {
    // Hollowing.cpp:94  `new Interior{}` relies on the default member initializers above.
    fn default() -> Self {
        Self {
            mesh: indexed_triangle_set::default(),
            gridptr: None,
            accessor: Cell::new(None),
            closing_distance: 0.0,
            thickness: 0.0,
            voxel_scale: 1.0,
            nb_in: 3.0,
            nb_out: 3.0,
        }
    }
}

impl Interior {
    /// Hollowing.cpp:37-42
    /// C++: `void reset_accessor() const` — This resets the accessor and its cache
    /// Not a thread safe call!
    pub fn reset_accessor(&self) {
        // Hollowing.cpp:40-41  if (gridptr) accessor = gridptr->getConstAccessor();
        if self.gridptr.is_some() {
            self.accessor.set(Some(()));
        }
    }
}

// Hollowing.cpp:45-48  void InteriorDeleter::operator()(Interior *p) { delete p; }
// (covered by `Box<Interior>`'s Drop — see the `InteriorPtr` note above)

/// Hollowing.cpp:50-53
/// C++: `indexed_triangle_set &get_mesh(Interior &interior)`
pub fn get_mesh_mut(interior: &mut Interior) -> &mut indexed_triangle_set {
    // Hollowing.cpp:52
    &mut interior.mesh
}

/// Hollowing.cpp:55-58
/// C++: `const indexed_triangle_set &get_mesh(const Interior &interior)`
pub fn get_mesh(interior: &Interior) -> &indexed_triangle_set {
    // Hollowing.cpp:57
    &interior.mesh
}

/// Hollowing.cpp:60-109
/// C++: `static InteriorPtr generate_interior_verbose(const TriangleMesh &mesh,
///       const JobController &ctl, double min_thickness, double voxel_scale,
///       double closing_dist)`
///
/// Returns `Ok(None)` on the C++ `return {}` stop-condition paths; `Err` when
/// hitting the blocked native OpenVDB boundary (where C++ would proceed).
fn generate_interior_verbose(
    mesh: &indexed_triangle_set,
    ctl: &JobController,
    min_thickness: f64,
    voxel_scale: f64,
    closing_dist: f64,
) -> Result<InteriorPtr> {
    // Hollowing.cpp:66  double offset = voxel_scale * min_thickness;
    let offset = voxel_scale * min_thickness;
    // Hollowing.cpp:67  double D = voxel_scale * closing_dist;
    let d = voxel_scale * closing_dist;
    // Hollowing.cpp:68  float out_range = 0.1f * float(offset);
    let out_range = 0.1f32 * offset as f32;
    // Hollowing.cpp:69  float in_range = 1.1f * float(offset + D);
    let in_range = 1.1f32 * (offset + d) as f32;

    // Hollowing.cpp:71-72
    if (ctl.stopcondition)() {
        return Ok(None);
    } else {
        (ctl.statuscb)(0, "Hollowing");
    }

    // Hollowing.cpp:74  auto gridptr = mesh_to_grid(mesh.its, {}, voxel_scale, out_range, in_range);
    // (BLOCKED native OpenVDB — `mesh_to_grid` below returns Err)
    #[allow(unused_mut)]
    let mut gridptr = mesh_to_grid(mesh, voxel_scale as f32, out_range, in_range)?;

    // Hollowing.cpp:76  assert(gridptr);
    // Hollowing.cpp:78-81  if (!gridptr) { BOOST_LOG_TRIVIAL(error) << "Returned
    // OpenVDB grid is NULL"; return {}; }
    // (the C++ null-grid path is subsumed by the Result above)

    // Hollowing.cpp:83-84
    if (ctl.stopcondition)() {
        return Ok(None);
    } else {
        (ctl.statuscb)(30, "Hollowing");
    }

    // Hollowing.cpp:86  double iso_surface = D;
    let iso_surface = d;
    // Hollowing.cpp:87  auto narrowb = double(in_range);
    let narrowb = in_range as f64;
    // Hollowing.cpp:88  gridptr = redistance_grid(*gridptr, -(offset + D), narrowb, narrowb);
    gridptr = redistance_grid(&gridptr, -(offset + d), narrowb, narrowb)?;

    // Hollowing.cpp:90-91
    if (ctl.stopcondition)() {
        return Ok(None);
    } else {
        (ctl.statuscb)(70, "Hollowing");
    }

    // Hollowing.cpp:93  double adaptivity = 0.;
    let adaptivity = 0.0;
    // Hollowing.cpp:94  InteriorPtr interior = InteriorPtr{new Interior{}};
    let mut interior: Box<Interior> = Box::new(Interior::default());

    // Hollowing.cpp:96  interior->mesh = grid_to_mesh(*gridptr, iso_surface, adaptivity);
    interior.mesh = grid_to_mesh(&gridptr, iso_surface, adaptivity)?;
    // Hollowing.cpp:97  interior->gridptr = gridptr;
    interior.gridptr = Some(gridptr);

    // Hollowing.cpp:99-100
    if (ctl.stopcondition)() {
        return Ok(None);
    } else {
        (ctl.statuscb)(100, "Hollowing");
    }

    // Hollowing.cpp:102-106
    interior.closing_distance = d;
    interior.thickness = offset;
    interior.voxel_scale = voxel_scale;
    interior.nb_in = narrowb;
    interior.nb_out = narrowb;

    // Hollowing.cpp:108
    Ok(Some(interior))
}

/// Hollowing.cpp:111-148
/// C++: `InteriorPtr generate_interior(const TriangleMesh &mesh,
///       const HollowingConfig &hc, const JobController &ctl)`
/// (defaults `hc = {}`, `ctl = {}` — Hollowing.hpp:73-75)
pub fn generate_interior(
    mesh: &indexed_triangle_set,
    hc: &HollowingConfig,
    ctl: &JobController,
) -> Result<InteriorPtr> {
    // Hollowing.cpp:115  static const double MIN_OVERSAMPL = 3.5;
    const MIN_OVERSAMPL: f64 = 3.5;
    // Hollowing.cpp:116  static const double MAX_OVERSAMPL = 8.;
    const MAX_OVERSAMPL: f64 = 8.0;

    // Hollowing.cpp:118-124
    // I can't figure out how to increase the grid resolution through openvdb
    // API so the model will be scaled up before conversion and the result
    // scaled down. Voxels have a unit size. If I set voxelSize smaller, it
    // scales the whole geometry down, and doesn't increase the number of
    // voxels.
    //
    // max 8x upscale, min is native voxel size
    // Hollowing.cpp:125
    let voxel_scale = MIN_OVERSAMPL + (MAX_OVERSAMPL - MIN_OVERSAMPL) * hc.quality;

    // Hollowing.cpp:127-129
    let mut interior =
        generate_interior_verbose(mesh, ctl, hc.min_thickness, voxel_scale, hc.closing_distance)?;

    // Hollowing.cpp:131  if (interior && !interior->mesh.empty()) {
    // (indexed_triangle_set::empty() == indices.empty() || vertices.empty(), admesh/stl.h:247)
    if let Some(interior) = interior.as_deref_mut() {
        if !(interior.mesh.indices.is_empty() || interior.mesh.vertices.is_empty()) {
            // Hollowing.cpp:133-134  flip normals back...
            swap_normals(&mut interior.mesh);

            // Hollowing.cpp:136-137  simplify mesh lossless
            // float loss_less_max_error = 2*std::numeric_limits<float>::epsilon();
            let mut loss_less_max_error: Option<f32> = Some(2.0 * f32::EPSILON);
            // Hollowing.cpp:138  its_quadric_edge_collapse(interior->mesh, 0U, &loss_less_max_error);
            its_quadric_edge_collapse_its(&mut interior.mesh, 0, &mut loss_less_max_error);

            // Hollowing.cpp:140  its_compactify_vertices(interior->mesh);
            // (C++ default shrink_to_fit = true, TriangleMesh.hpp:217)
            its_compactify_vertices(&mut interior.mesh, true);
            // Hollowing.cpp:141  its_merge_vertices(interior->mesh);
            // (C++ default shrink_to_fit = true, TriangleMesh.hpp:211)
            its_merge_vertices(&mut interior.mesh, true);

            // Hollowing.cpp:143-144  flip normals back...
            swap_normals(&mut interior.mesh);
        }
    }

    // Hollowing.cpp:147
    Ok(interior)
}

/// Bridge to the crate's `its_quadric_edge_collapse` port, which operates on
/// `crate::triangle_mesh::TriangleMesh` (f64 `Point3F` vertices / `Triangle`
/// indices) instead of `indexed_triangle_set`. The conversion is loss-less for
/// the inputs (f32 -> f64 widening); the QEC port computes new vertex positions
/// in f64 where C++ uses f32 (pre-existing divergence of that port).
fn its_quadric_edge_collapse_its(
    its: &mut indexed_triangle_set,
    triangle_count: u32,
    max_error: &mut Option<f32>,
) {
    let vertices: Vec<Point3F> = its
        .vertices
        .iter()
        .map(|v| Point3F::new(v.x as f64, v.y as f64, v.z as f64))
        .collect();
    let indices: Vec<Triangle> = its
        .indices
        .iter()
        .map(|t| Triangle::new(t[0] as u32, t[1] as u32, t[2] as u32))
        .collect();
    let mut tm = TriangleMesh::from_parts(vertices, indices);
    // QuadricEdgeCollapse.hpp:32 — throw_on_cancel / status_fn default to nullptr.
    its_quadric_edge_collapse(&mut tm, triangle_count, max_error, None, None);
    its.vertices = tm
        .vertices()
        .iter()
        .map(|p| Vec3f::new(p.x as f32, p.y as f32, p.z as f32))
        .collect();
    its.indices = tm
        .indices()
        .iter()
        .map(|t| Vec3i::new(t.indices[0] as i32, t.indices[1] as i32, t.indices[2] as i32))
        .collect();
}

impl DrainHole {
    /// Hollowing.cpp:150-160
    /// C++: `indexed_triangle_set DrainHole::to_mesh() const`
    pub fn to_mesh(&self) -> indexed_triangle_set {
        // Hollowing.cpp:152  auto r = double(radius);
        let r = self.radius as f64;
        // Hollowing.cpp:153  auto h = double(height);
        let h = self.height as f64;
        // Hollowing.cpp:154  indexed_triangle_set hole = sla::cylinder(r, h, steps);
        // (default `sp = Vec3d::Zero()`, SupportTreeMesher.hpp:28-31)
        let mut hole = cylinder(r, h, Self::STEPS, &GeoVec3d::zero());
        // Hollowing.cpp:155-156
        // Eigen::Quaternionf q; q.setFromTwoVectors(Vec3f{0.f, 0.f, 1.f}, normal);
        let q = Quaternionf::set_from_two_vectors(&Vec3f::new(0.0, 0.0, 1.0), &self.normal);
        // Hollowing.cpp:157  for(auto& p : hole.vertices) p = q * p + pos;
        for p in &mut hole.vertices {
            *p = q.transform_vector(p) + self.pos;
        }

        // Hollowing.cpp:159
        hole
    }

    // Hollowing.cpp:162-167  bool DrainHole::operator==(const DrainHole &sp) const
    // (ported as the `PartialEq` impl above)

    /// Hollowing.cpp:169-181
    /// C++: `bool DrainHole::is_inside(const Vec3f& pt) const`
    pub fn is_inside(&self, pt: &Vec3f) -> bool {
        // Hollowing.cpp:171  Eigen::Hyperplane<float, 3> plane(normal, pos);
        let plane = Hyperplane3f::new(self.normal, self.pos);
        // Hollowing.cpp:172  float dist = plane.signedDistance(pt);
        let dist = plane.signed_distance(pt);
        // Hollowing.cpp:173-174
        if dist < EPSILON as f32 || dist > self.height {
            return false;
        }

        // Hollowing.cpp:176  Eigen::ParametrizedLine<float, 3> axis(pos, normal);
        let axis = ParametrizedLine3f::new(self.pos, self.normal);
        // Hollowing.cpp:177-178  if (axis.squaredDistance(pt) < pow(radius, 2.f)) return true;
        if axis.squared_distance(pt) < self.radius.powf(2.0) {
            return true;
        }

        // Hollowing.cpp:180
        false
    }

    /// Hollowing.cpp:184-278
    /// Given a line s+dir*t, find parameter t of intersections with the hole
    /// and the normal (points inside the hole). Outputs through out reference,
    /// returns true if two intersections were found.
    /// C++: `bool DrainHole::get_intersections(const Vec3f& s, const Vec3f& dir,
    ///       std::array<std::pair<float, Vec3d>, 2>& out) const`
    pub fn get_intersections(
        &self,
        s: &Vec3f,
        dir: &Vec3f,
        out: &mut [(f32, Vec3d); 2],
    ) -> bool {
        // Hollowing.cpp:191  assert(is_approx(normal.norm(), 1.f));
        debug_assert!(is_approx_f32(self.normal.norm(), 1.0));
        // Hollowing.cpp:192  const Eigen::ParametrizedLine<float, 3> ray(s, dir.normalized());
        let ray = ParametrizedLine3f::new(*s, dir.normalize());

        // Hollowing.cpp:194-195
        for i in 0..2 {
            out[i] = (hit_result::infty() as f32, Vec3d::zeros());
        }

        // Hollowing.cpp:197  const float sqr_radius = pow(radius, 2.f);
        let sqr_radius = self.radius.powf(2.0);

        // first check a bounding sphere of the hole: (Hollowing.cpp:199)
        // Hollowing.cpp:200  Vec3f center = pos+normal*height/2.f;
        let center = self.pos + self.normal * self.height / 2.0;
        // Hollowing.cpp:201  float sqr_dist_limit = pow(height/2.f, 2.f) + sqr_radius;
        let sqr_dist_limit = (self.height / 2.0).powf(2.0) + sqr_radius;
        // Hollowing.cpp:202-203
        if ray.squared_distance(&center) > sqr_dist_limit {
            return false;
        }

        // The line intersects the bounding sphere, look for intersections with
        // bases of the cylinder. (Hollowing.cpp:205-206)

        // Hollowing.cpp:208  size_t found = 0; // counts how many intersections were found
        let mut found: usize = 0;
        // Hollowing.cpp:209  Eigen::Hyperplane<float, 3> base;
        // (default-constructed/uninitialized in C++; always assigned before use)
        let mut base = Hyperplane3f {
            normal: Vec3f::zeros(),
            offset: 0.0,
        };
        // Hollowing.cpp:210  if (! is_approx(ray.direction().dot(normal), 0.f)) {
        if !is_approx_f32(ray.direction.dot(&self.normal), 0.0) {
            // Hollowing.cpp:211  for (size_t i=1; i<=1; --i) {
            // (size_t wrap-around: iterates i = 1, then i = 0)
            for i in (0..=1usize).rev() {
                // Hollowing.cpp:212  Vec3f cylinder_center = pos+i*height*normal;
                let mut cylinder_center = self.pos + (i as f32) * self.height * self.normal;
                // Hollowing.cpp:213-217
                if i == 0 {
                    // The hole base can be identical to mesh surface if it is flat
                    // let's better move the base outward a bit
                    cylinder_center -= (EPSILON as f32) * self.normal;
                }
                // Hollowing.cpp:218  base = Eigen::Hyperplane<float, 3>(normal, cylinder_center);
                base = Hyperplane3f::new(self.normal, cylinder_center);
                // Hollowing.cpp:219  Vec3f intersection = ray.intersectionPoint(base);
                let intersection = ray.intersection_point(&base);
                // Only accept the point if it is inside the cylinder base. (Hollowing.cpp:220)
                // Hollowing.cpp:221
                if (cylinder_center - intersection).norm_squared() < sqr_radius {
                    // Hollowing.cpp:222
                    out[found].0 = ray.intersection_parameter(&base);
                    // Hollowing.cpp:223
                    out[found].1 =
                        (if i == 0 { 1.0 } else { -1.0 }) * self.normal.cast::<f64>();
                    // Hollowing.cpp:224
                    found += 1;
                }
            }
        } else {
            // In case the line was perpendicular to the cylinder axis, previous
            // block was skipped, but base will later be assumed to be valid.
            // (Hollowing.cpp:230-231)
            // Hollowing.cpp:232
            base = Hyperplane3f::new(self.normal, self.pos - (EPSILON as f32) * self.normal);
        }

        // In case there is still an intersection to be found, check the wall
        // (Hollowing.cpp:235)
        // Hollowing.cpp:236
        if found != 2 && !is_approx_f32(ray.direction.dot(&self.normal).abs(), 1.0) {
            // Project the ray onto the base plane (Hollowing.cpp:237)
            // Hollowing.cpp:238  Vec3f proj_origin = base.projection(ray.origin());
            let proj_origin = base.projection(&ray.origin);
            // Hollowing.cpp:239
            let mut proj_dir = base.projection(&(ray.origin + ray.direction)) - proj_origin;
            // save how the parameter scales and normalize the projected direction
            // (Hollowing.cpp:240)
            // Hollowing.cpp:241  float par_scale = proj_dir.norm();
            let par_scale = proj_dir.norm();
            // Hollowing.cpp:242  proj_dir = proj_dir/par_scale;
            proj_dir = proj_dir / par_scale;
            // Hollowing.cpp:243
            let projected_ray = ParametrizedLine3f::new(proj_origin, proj_dir);
            // Calculate point on the secant that's closest to the center
            // and its distance to the circle along the projected line
            // (Hollowing.cpp:244-245)
            // Hollowing.cpp:246  Vec3f closest = projected_ray.projection(pos);
            let closest = projected_ray.projection(&self.pos);
            // Hollowing.cpp:247  float dist = sqrt((sqr_radius - (closest-pos).squaredNorm()));
            let dist = (sqr_radius - (closest - self.pos).norm_squared()).sqrt();

            // Unproject both intersections on the original line and check
            // they are on the cylinder and not past it: (Hollowing.cpp:248-249)
            // Hollowing.cpp:250  for (int i=-1; i<=1 && found !=2; i+=2) {
            let mut i: i32 = -1;
            while i <= 1 && found != 2 {
                // Hollowing.cpp:251  Vec3f isect = closest + i*dist * projected_ray.direction();
                let mut isect = closest + (i as f32) * dist * projected_ray.direction;
                // Hollowing.cpp:252  Vec3f to_isect = isect-proj_origin;
                let to_isect = isect - proj_origin;
                // Hollowing.cpp:253  float par = to_isect.norm() / par_scale;
                let mut par = to_isect.norm() / par_scale;
                // Hollowing.cpp:254-255
                if to_isect.normalize().dot(&proj_dir.normalize()) < 0.0 {
                    par *= -1.0;
                }
                // Hollowing.cpp:256  Vec3d hit_normal = (pos-isect).normalized().cast<double>();
                let hit_normal = (self.pos - isect).normalize().cast::<f64>();
                // Hollowing.cpp:257  isect = ray.pointAt(par);
                isect = ray.point_at(par);
                // check that the intersection is between the base planes:
                // (Hollowing.cpp:258)
                // Hollowing.cpp:259  float vert_dist = base.signedDistance(isect);
                let vert_dist = base.signed_distance(&isect);
                // Hollowing.cpp:260
                if vert_dist > 0.0 && vert_dist < self.height {
                    // Hollowing.cpp:261-263
                    out[found].0 = par;
                    out[found].1 = hit_normal;
                    found += 1;
                }
                i += 2;
            }
        }

        // If only one intersection was found, it is some corner case,
        // no intersection will be returned: (Hollowing.cpp:268-269)
        // Hollowing.cpp:270-271
        if found != 2 {
            return false;
        }

        // Sort the intersections: (Hollowing.cpp:273)
        // Hollowing.cpp:274-275  if (out[0].first > out[1].first) std::swap(out[0], out[1]);
        if out[0].0 > out[1].0 {
            out.swap(0, 1);
        }

        // Hollowing.cpp:277
        true
    }
}

/// Hollowing.cpp:280-302
/// C++: `void cut_drainholes(std::vector<ExPolygons> &obj_slices,
///       const std::vector<float> &slicegrid, float closing_radius,
///       const sla::DrainHoles &holes, std::function<void(void)> thr)`
pub fn cut_drainholes(
    obj_slices: &mut [ExPolygons],
    slicegrid: &[f32],
    closing_radius: f32,
    holes: &DrainHoles,
    thr: &dyn Fn(),
) {
    // Hollowing.cpp:286  TriangleMesh mesh;
    let mut mesh = indexed_triangle_set::default();
    // Hollowing.cpp:287-288
    // for (const sla::DrainHole &holept : holes) mesh.merge(TriangleMesh{holept.to_mesh()});
    for holept in holes {
        its_merge(&mut mesh, &holept.to_mesh());
    }

    // Hollowing.cpp:290  if (mesh.empty()) return;
    // (TriangleMesh::empty() == facets_count() == 0, TriangleMesh.hpp:142)
    if mesh.indices.is_empty() {
        return;
    }

    // Hollowing.cpp:292
    // std::vector<ExPolygons> hole_slices = slice_mesh_ex(mesh.its, slicegrid, closing_radius, thr);
    // (inline overload TriangleMeshSlicer.hpp:86-95: params.closing_radius = closing_radius)
    let mut params = MeshSlicingParamsEx::default();
    params.closing_radius = closing_radius;
    let hole_slices = slice_mesh_ex_its(&mesh, slicegrid, &params, thr);

    // Hollowing.cpp:294-296
    if obj_slices.len() != hole_slices.len() {
        log::warn!("Sliced object and drain-holes layer count does not match!");
    }

    // Hollowing.cpp:298  size_t until = std::min(obj_slices.size(), hole_slices.size());
    let until = obj_slices.len().min(hole_slices.len());

    // Hollowing.cpp:300-301
    // for (size_t i = 0; i < until; ++i) obj_slices[i] = diff_ex(obj_slices[i], hole_slices[i]);
    for i in 0..until {
        obj_slices[i] = difference(&obj_slices[i], &hole_slices[i]);
    }
}

/// Hollowing.cpp:304-310
/// C++: `void hollow_mesh(TriangleMesh &mesh, const HollowingConfig &cfg, int flags)`
/// (default `flags = 0`, Hollowing.hpp:78)
pub fn hollow_mesh(mesh: &mut indexed_triangle_set, cfg: &HollowingConfig, flags: i32) -> Result<()> {
    // Hollowing.cpp:306  InteriorPtr interior = generate_interior(mesh, cfg, JobController{});
    let interior = generate_interior(mesh, cfg, &JobController::default())?;
    // Hollowing.cpp:307  if (!interior) return;
    let interior = match interior {
        Some(interior) => interior,
        None => return Ok(()),
    };

    // Hollowing.cpp:309  hollow_mesh(mesh, *interior, flags);
    hollow_mesh_interior(mesh, &interior, flags)
}

/// Hollowing.cpp:312-320
/// C++: `void hollow_mesh(TriangleMesh &mesh, const Interior &interior, int flags)`
/// (overload — Hollowing prepared in "interior", merge with original mesh;
/// default `flags = 0`, Hollowing.hpp:81)
pub fn hollow_mesh_interior(
    mesh: &mut indexed_triangle_set,
    interior: &Interior,
    flags: i32,
) -> Result<()> {
    // Hollowing.cpp:314  if (mesh.empty() || interior.mesh.empty()) return;
    // (TriangleMesh::empty() == indices.empty(); its::empty() == indices || vertices empty)
    if mesh.indices.is_empty()
        || (interior.mesh.indices.is_empty() || interior.mesh.vertices.is_empty())
    {
        return Ok(());
    }

    // Hollowing.cpp:316-317
    // if (flags & hfRemoveInsideTriangles && interior.gridptr) remove_inside_triangles(mesh, interior);
    if (flags & HollowingFlags::hfRemoveInsideTriangles as i32) != 0 && interior.gridptr.is_some()
    {
        remove_inside_triangles(mesh, interior, &[])?;
    }

    // Hollowing.cpp:319  mesh.merge(TriangleMesh{interior.mesh});
    its_merge(mesh, &interior.mesh);
    Ok(())
}

/// Hollowing.cpp:322-336
/// Get the distance of p to the interior's zero iso_surface. Interior should
/// have its zero isosurface positioned at offset + closing_distance inwards form
/// the model surface.
/// C++: `static double get_distance_raw(const Vec3f &p, const Interior &interior)`
///
/// BLOCKED native: the voxel lookup (`worldToIndexCellCentered` +
/// `ConstAccessor::getValue`) requires the native OpenVDB grid; returns `Err`.
fn get_distance_raw(p: &Vec3f, interior: &Interior) -> Result<f64> {
    // Hollowing.cpp:327  assert(interior.gridptr);
    debug_assert!(interior.gridptr.is_some());

    // Hollowing.cpp:329  if (!interior.accessor) interior.reset_accessor();
    if interior.accessor.get().is_none() {
        interior.reset_accessor();
    }

    // Hollowing.cpp:331  auto v = (p * interior.voxel_scale).cast<double>();
    // (Eigen converts the double scalar to float for the Vec3f product)
    let _v: Vec3d = (p * interior.voxel_scale as f32).cast::<f64>();

    // Hollowing.cpp:332-333  auto grididx = interior.gridptr->transform()
    //     .worldToIndexCellCentered({v.x(), v.y(), v.z()});
    // Hollowing.cpp:335  return interior.accessor->getValue(grididx);
    Err(Error::Mesh(
        "sla::hollowing::get_distance_raw (Hollowing.cpp:332-335): blocked on the native \
         OpenVDB FloatGrid accessor (openvdb::FloatGrid::ConstAccessor::getValue); \
         no pure-Rust/wasm-safe backend exists"
            .into(),
    ))
}

/// Hollowing.cpp:338  struct TriangleBubble { Vec3f center; double R; };
struct TriangleBubble {
    center: Vec3f,
    // C++ member `R` (renamed per snake_case convention)
    r: f64,
}

/// Hollowing.cpp:340-356
/// Return the distance of bubble center to the interior boundary or NaN if the
/// triangle is too big to be measured.
/// C++: `static double get_distance(const TriangleBubble &b, const Interior &interior)`
fn get_distance_bubble(b: &TriangleBubble, interior: &Interior) -> Result<f64> {
    // Hollowing.cpp:344  double R = b.R * interior.voxel_scale;
    let r = b.r * interior.voxel_scale;
    // Hollowing.cpp:345  double D = get_distance_raw(b.center, interior);
    let d = get_distance_raw(&b.center, interior)?;

    // Hollowing.cpp:347-355
    Ok(
        if (d > 0.0 && r >= interior.nb_out)
            || (d < 0.0 && r >= interior.nb_in)
            || ((d - r) < 0.0 && 2.0 * r > interior.thickness)
        {
            // std::nan("")
            f64::NAN
        } else {
            // FIXME: Adding interior.voxel_scale is a compromise supposed
            // to prevent the deletion of the triangles forming the interior
            // itself. This has a side effect that a small portion of the
            // bad triangles will still be visible.
            // Hollowing.cpp:355  D - interior.closing_distance /*+ 2 * interior.voxel_scale*/;
            d - interior.closing_distance
        },
    )
}

/// Hollowing.cpp:358-362
/// C++: `double get_distance(const Vec3f &p, const Interior &interior)`
pub fn get_distance(p: &Vec3f, interior: &Interior) -> Result<f64> {
    // Hollowing.cpp:360  double d = get_distance_raw(p, interior) - interior.closing_distance;
    let d = get_distance_raw(p, interior)? - interior.closing_distance;
    // Hollowing.cpp:361  return d / interior.voxel_scale;
    Ok(d / interior.voxel_scale)
}

/// Hollowing.hpp:88-92
/// C++: `template<class T> FloatingOnly<T> get_distance(const Vec<3, T> &p,
///       const Interior &interior) { return get_distance(Vec3f(p.template
///       cast<float>()), interior); }` — instantiated for `double`.
pub fn get_distance_vec3d(p: &Vec3d, interior: &Interior) -> Result<f64> {
    get_distance(&p.cast::<f32>(), interior)
}

// Hollowing.cpp:364-366
// A face that can be divided. Stores the indices into the original mesh if its
// part of that mesh and the vertices it consists of.
/// Hollowing.cpp:366  enum { NEW_FACE = -1};
const NEW_FACE: i64 = -1;

/// Hollowing.cpp:367-372  struct DivFace
struct DivFace {
    // Hollowing.cpp:368  Vec3i indx;
    indx: Vec3i,
    // Hollowing.cpp:369  std::array<Vec3f, 3> verts;
    verts: [Vec3f; 3],
    // Hollowing.cpp:370  long faceid = NEW_FACE;
    faceid: i64,
    // Hollowing.cpp:371  long parent = NEW_FACE;
    parent: i64,
}

/// Hollowing.cpp:374-411
/// Divide a face recursively and call visitor on all the sub-faces.
/// C++: `template<class Fn> void divide_triangle(const DivFace &face, Fn &&visitor)`
fn divide_triangle<F: FnMut(&DivFace) -> bool>(face: &DivFace, visitor: &mut F) {
    // Hollowing.cpp:378-380
    let edges: [Vec3f; 3] = [
        face.verts[0] - face.verts[1],
        face.verts[1] - face.verts[2],
        face.verts[2] - face.verts[0],
    ];

    // Hollowing.cpp:382  std::array<size_t, 3> edgeidx = {0, 1, 2};
    let mut edgeidx: [usize; 3] = [0, 1, 2];

    // Hollowing.cpp:384-386
    // std::sort(..., [&edges](size_t e1, size_t e2) {
    //     return edges[e1].squaredNorm() > edges[e2].squaredNorm(); });
    edgeidx.sort_unstable_by(|&e1, &e2| {
        edges[e2]
            .norm_squared()
            .partial_cmp(&edges[e1].norm_squared())
            .unwrap()
    });

    // Hollowing.cpp:388  DivFace child1, child2;

    // Hollowing.cpp:390-396
    let child1 = DivFace {
        // child1.parent = face.faceid == NEW_FACE ? face.parent : face.faceid;
        parent: if face.faceid == NEW_FACE {
            face.parent
        } else {
            face.faceid
        },
        // child1.indx(0) = -1;
        // child1.indx(1) = face.indx(edgeidx[1]);
        // child1.indx(2) = face.indx((edgeidx[1] + 1) % 3);
        indx: Vec3i::new(
            -1,
            face.indx[edgeidx[1]],
            face.indx[(edgeidx[1] + 1) % 3],
        ),
        // child1.verts[0] = (face.verts[edgeidx[0]] + face.verts[(edgeidx[0] + 1) % 3]) / 2.;
        // child1.verts[1] = face.verts[edgeidx[1]];
        // child1.verts[2] = face.verts[(edgeidx[1] + 1) % 3];
        verts: [
            (face.verts[edgeidx[0]] + face.verts[(edgeidx[0] + 1) % 3]) / 2.0,
            face.verts[edgeidx[1]],
            face.verts[(edgeidx[1] + 1) % 3],
        ],
        // (faceid keeps the DivFace default NEW_FACE, Hollowing.cpp:370)
        faceid: NEW_FACE,
    };

    // Hollowing.cpp:398-399
    if visitor(&child1) {
        divide_triangle(&child1, visitor);
    }

    // Hollowing.cpp:401-407
    let child2 = DivFace {
        // child2.parent = face.faceid == NEW_FACE ? face.parent : face.faceid;
        parent: if face.faceid == NEW_FACE {
            face.parent
        } else {
            face.faceid
        },
        // child2.indx(0) = -1;
        // child2.indx(1) = face.indx(edgeidx[2]);
        // child2.indx(2) = face.indx((edgeidx[2] + 1) % 3);
        indx: Vec3i::new(
            -1,
            face.indx[edgeidx[2]],
            face.indx[(edgeidx[2] + 1) % 3],
        ),
        // child2.verts[0] = child1.verts[0];
        // child2.verts[1] = face.verts[edgeidx[2]];
        // child2.verts[2] = face.verts[(edgeidx[2] + 1) % 3];
        verts: [
            child1.verts[0],
            face.verts[edgeidx[2]],
            face.verts[(edgeidx[2] + 1) % 3],
        ],
        faceid: NEW_FACE,
    };

    // Hollowing.cpp:409-410
    if visitor(&child2) {
        divide_triangle(&child2, visitor);
    }
}

/// `BoundingBoxf3 facebb { pts.begin(), pts.end() };` (Hollowing.cpp:469, :523)
/// — BoundingBox3Base built from Vec3f points (merged as doubles).
fn bounding_boxf3_of_verts(pts: &[Vec3f; 3]) -> BoundingBoxf3 {
    let mut bb = BoundingBoxf3::new();
    for p in pts {
        bb.merge_point(GeoVec3d::new(p.x as f64, p.y as f64, p.z as f64));
    }
    bb
}

/// Hollowing.cpp:413-561
/// C++: `void remove_inside_triangles(TriangleMesh &mesh, const Interior &interior,
///       const std::vector<bool> &exclude_mask)`
/// (default `exclude_mask = {}`, Hollowing.hpp:83-84)
///
/// Returns `Err` (before modifying the mesh) if the interior's grid distance
/// queries hit the blocked native OpenVDB boundary; C++ has no such path.
pub fn remove_inside_triangles(
    mesh: &mut indexed_triangle_set,
    interior: &Interior,
    exclude_mask: &[bool],
) -> Result<()> {
    // Hollowing.cpp:416  enum TrPos { posInside, posTouch, posOutside };
    // (declared but never used in the C++ function body)

    // Hollowing.cpp:418-419  auto &faces = mesh.its.indices; auto &vertices = mesh.its.vertices;
    // (accessed through `mesh.indices` / `mesh.vertices` below)
    // Hollowing.cpp:420  auto bb = mesh.bounding_box();
    // (TriangleMesh::bounding_box() == BoundingBoxf3 of the vertices, TriangleMesh.cpp)
    let bb = {
        let mut bb = BoundingBoxf3::new();
        for v in &mesh.vertices {
            bb.merge_point(GeoVec3d::new(v.x as f64, v.y as f64, v.z as f64));
        }
        bb
    };

    // Hollowing.cpp:422  bool use_exclude_mask = faces.size() == exclude_mask.size();
    let use_exclude_mask = mesh.indices.len() == exclude_mask.len();
    // Hollowing.cpp:423-425
    let is_excluded =
        |face_id: usize| -> bool { use_exclude_mask && exclude_mask[face_id] };

    // Hollowing.cpp:427-428  // TODO: Parallel mode not working yet
    // using exec_policy = ccr_seq;
    // (`ccr_seq::for_each` executes the loop body sequentially in index order;
    // ported as a plain loop below. Its `SpinningMutex` is the sequential no-op
    // `_Mtx`, so `MeshMods.new_triangles` needs no lock here.)

    // Hollowing.cpp:430-465  struct MeshMods — info about the needed
    // modifications on the input mesh.
    struct MeshMods {
        // Hollowing.cpp:433-450  Just a thread safe wrapper for a vector of
        // triangles. (mutex elided: sequential policy, see above)
        new_triangles: Vec<[Vec3f; 3]>,

        // Hollowing.cpp:452-454  A vector of bool for all faces signaling if it
        // needs to be removed or not.
        to_remove: Vec<bool>,
    }

    impl MeshMods {
        // Hollowing.cpp:456-457
        // MeshMods(const TriangleMesh &mesh): to_remove(mesh.its.indices.size(), false) {}
        fn new(mesh: &indexed_triangle_set) -> Self {
            Self {
                new_triangles: Vec::new(),
                to_remove: vec![false; mesh.indices.len()],
            }
        }

        // Hollowing.cpp:459-463  Number of triangles that need to be removed.
        fn to_remove_cnt(&self) -> usize {
            // std::accumulate(to_remove.begin(), to_remove.end(), size_t(0));
            self.to_remove.iter().map(|&b| b as usize).sum()
        }
    }

    // Hollowing.cpp:465  } mesh_mods{mesh};
    let mut mesh_mods = MeshMods::new(mesh);

    // First error from the (native-blocked) distance queries, propagated after
    // the loop. (No C++ counterpart — get_distance cannot fail there.)
    let mut dist_err: Option<Error> = None;

    // Hollowing.cpp:467-509
    // Must return true if further division of the face is needed.
    let mut divfn = |f: &DivFace| -> bool {
        // Hollowing.cpp:469  BoundingBoxf3 facebb { f.verts.begin(), f.verts.end() };
        let facebb = bounding_boxf3_of_verts(&f.verts);

        // Face is certainly outside the cavity (Hollowing.cpp:471)
        // Hollowing.cpp:472-474
        if !facebb.intersects(&bb) && f.faceid != NEW_FACE {
            return false;
        }

        // Hollowing.cpp:476
        // TriangleBubble bubble{facebb.center().cast<float>(), facebb.radius()};
        let c = facebb.center();
        let bubble = TriangleBubble {
            center: Vec3f::new(c.x as f32, c.y as f32, c.z as f32),
            r: facebb.radius(),
        };

        // Hollowing.cpp:478  double D = get_distance(bubble, interior);
        let d = match get_distance_bubble(&bubble, interior) {
            Ok(d) => d,
            Err(e) => {
                // Native-blocked: record the error and stop processing this face.
                if dist_err.is_none() {
                    dist_err = Some(e);
                }
                return false;
            }
        };
        // Hollowing.cpp:479  double R = bubble.R * interior.voxel_scale;
        let r = bubble.r * interior.voxel_scale;

        // Hollowing.cpp:481-482
        if d.is_nan() {
            // The distance cannot be measured, triangle too big
            return true;
        }

        // Distance of the bubble wall to the interior wall. Negative if the
        // bubble is overlapping with the interior (Hollowing.cpp:484-485)
        // Hollowing.cpp:486
        let bubble_distance = d - r;

        // The face is crossing the interior or inside, it must be removed and
        // parts of it re-added, that are outside the interior
        // (Hollowing.cpp:488-489)
        // Hollowing.cpp:490
        if bubble_distance < 0.0 {
            // Hollowing.cpp:491-492
            if f.faceid != NEW_FACE {
                mesh_mods.to_remove[f.faceid as usize] = true;
            }

            // Hollowing.cpp:494-495  Top parent needs to be removed as well
            if f.parent != NEW_FACE {
                mesh_mods.to_remove[f.parent as usize] = true;
            }

            // If the outside part is between the interior end the exterior
            // (inside the wall being invisible), no further division is needed.
            // (Hollowing.cpp:497-498)
            // Hollowing.cpp:499-500
            if (r + d) < interior.thickness {
                return false;
            }

            // Hollowing.cpp:502
            return true;
        } else if f.faceid == NEW_FACE {
            // New face completely outside needs to be re-added.
            // (Hollowing.cpp:504-505)
            mesh_mods.new_triangles.push(f.verts);
        }

        // Hollowing.cpp:508
        false
    };

    // Hollowing.cpp:511
    interior.reset_accessor();

    // Hollowing.cpp:513-533
    // exec_policy::for_each(size_t(0), faces.size(), ..., exec_policy::max_concurreny());
    // (ccr_seq: sequential in-order iteration)
    for face_idx in 0..mesh.indices.len() {
        // Hollowing.cpp:514  const Vec3i &face = faces[face_idx];
        let face = mesh.indices[face_idx];

        // If the triangle is excluded, we need to keep it. (Hollowing.cpp:516)
        // Hollowing.cpp:517-518
        if is_excluded(face_idx) {
            continue;
        }

        // Hollowing.cpp:520-521
        // std::array<Vec3f, 3> pts = { vertices[face(0)], vertices[face(1)], vertices[face(2)] };
        let pts: [Vec3f; 3] = [
            mesh.vertices[face[0] as usize],
            mesh.vertices[face[1] as usize],
            mesh.vertices[face[2] as usize],
        ];

        // Hollowing.cpp:523  BoundingBoxf3 facebb { pts.begin(), pts.end() };
        let facebb = bounding_boxf3_of_verts(&pts);

        // Face is certainly outside the cavity (Hollowing.cpp:525)
        // Hollowing.cpp:526
        if !facebb.intersects(&bb) {
            continue;
        }

        // Hollowing.cpp:528  DivFace df{face, pts, long(face_idx)};
        let df = DivFace {
            indx: face,
            verts: pts,
            faceid: face_idx as i64,
            parent: NEW_FACE,
        };

        // Hollowing.cpp:530-531
        if divfn(&df) {
            divide_triangle(&df, &mut divfn);
        }
    }

    // Propagate the native-blocked distance error before mutating the mesh
    // (the collected modifications would be meaningless without distances).
    if let Some(e) = dist_err {
        return Err(e);
    }

    // Hollowing.cpp:535-536
    // auto new_faces = reserve_vector<Vec3i>(faces.size() + mesh_mods.new_triangles.size());
    let mut new_faces: Vec<Vec3i> =
        Vec::with_capacity(mesh.indices.len() + mesh_mods.new_triangles.len());

    // Hollowing.cpp:538-541
    for face_idx in 0..mesh.indices.len() {
        if !mesh_mods.to_remove[face_idx] {
            new_faces.push(mesh.indices[face_idx]);
        }
    }

    // Hollowing.cpp:543-549
    for i in 0..mesh_mods.new_triangles.len() {
        // size_t o = vertices.size();
        let o = mesh.vertices.len();
        mesh.vertices.push(mesh_mods.new_triangles[i][0]);
        mesh.vertices.push(mesh_mods.new_triangles[i][1]);
        mesh.vertices.push(mesh_mods.new_triangles[i][2]);
        // new_faces.emplace_back(int(o), int(o + 1), int(o + 2));
        new_faces.push(Vec3i::new(o as i32, (o + 1) as i32, (o + 2) as i32));
    }

    // Hollowing.cpp:551-554
    log::info!(
        "Trimming: {} triangles removed",
        mesh_mods.to_remove_cnt()
    );
    log::info!(
        "Trimming: {} triangles added",
        mesh_mods.new_triangles.len()
    );

    // Hollowing.cpp:556-557  faces.swap(new_faces); new_faces = {};
    mesh.indices = new_faces;

    // Hollowing.cpp:559  mesh = TriangleMesh{mesh.its};
    // (re-wraps the same indexed_triangle_set; a no-op at the its level)
    // FIXME do we want to repair the mesh? Are there duplicate vertices or
    // flipped triangles? (Hollowing.cpp:560)
    Ok(())
}

// ============================================================================
// BLOCKED native OpenVDB boundary (OpenVDBUtils.hpp symbols used by this file)
//
// These are the `OpenVDBUtils.hpp` grid functions invoked by
// `generate_interior_verbose`. They are thin wrappers over native OpenVDB
// (openvdb::tools::meshToVolume / levelSetRebuild / volumeToMesh) which cannot
// be ported (not wasm-safe; no pure-Rust equivalent). Full faithful commented
// bodies live in `src/open_vdb_utils.rs`. Each returns an explicit `Err`
// rather than fake data, mirroring `csg_mesh/voxelize_csg_mesh.rs`.
// ============================================================================

/// OpenVDBUtils.hpp:29-34 / OpenVDBUtils.cpp:48-87
/// `mesh_to_grid(mesh.its, {} /*transform*/, voxel_scale, exteriorBandWidth,
/// interiorBandWidth)` as called at Hollowing.cpp:74.
fn mesh_to_grid(
    _mesh: &indexed_triangle_set,
    _voxel_scale: f32,
    _exterior_band_width: f32,
    _interior_band_width: f32,
) -> Result<VoxelGrid> {
    Err(Error::Mesh(
        "sla::hollowing::mesh_to_grid (OpenVDBUtils.cpp:48-87): blocked on the native OpenVDB \
         backend (openvdb::tools::meshToVolume); no pure-Rust/wasm-safe port exists"
            .into(),
    ))
}

/// OpenVDBUtils.hpp:41-42 / OpenVDBUtils.cpp:122-134
/// `redistance_grid(*gridptr, -(offset + D), narrowb, narrowb)` as called at
/// Hollowing.cpp:88.
fn redistance_grid(_grid: &VoxelGrid, _iso: f64, _er: f64, _ir: f64) -> Result<VoxelGrid> {
    Err(Error::Mesh(
        "sla::hollowing::redistance_grid (OpenVDBUtils.cpp:122-134): blocked on the native \
         OpenVDB backend (openvdb::tools::levelSetRebuild); no pure-Rust/wasm-safe port exists"
            .into(),
    ))
}

/// OpenVDBUtils.hpp:36-39 / OpenVDBUtils.cpp:89-120
/// `grid_to_mesh(*gridptr, iso_surface, adaptivity)` as called at
/// Hollowing.cpp:96.
fn grid_to_mesh(
    _grid: &VoxelGrid,
    _isovalue: f64,
    _adaptivity: f64,
) -> Result<indexed_triangle_set> {
    Err(Error::Mesh(
        "sla::hollowing::grid_to_mesh (OpenVDBUtils.cpp:89-120): blocked on the native OpenVDB \
         backend (openvdb::tools::volumeToMesh); no pure-Rust/wasm-safe port exists"
            .into(),
    ))
}

// Hollowing.cpp:563  }} // namespace Slic3r::sla

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hollowing_config_defaults() {
        // Hollowing.hpp:14-17
        let cfg = HollowingConfig::default();
        assert_eq!(cfg.min_thickness, 2.0);
        assert_eq!(cfg.quality, 0.5);
        assert_eq!(cfg.closing_distance, 0.5);
        assert!(cfg.enabled);
    }

    #[test]
    fn drain_hole_defaults_and_eq() {
        // Hollowing.hpp:39-41
        let d = DrainHole::default();
        assert_eq!(d.pos, Vec3f::zeros());
        assert_eq!(d.normal, Vec3f::new(0.0, 0.0, 1.0));
        assert_eq!(d.radius, 5.0);
        assert_eq!(d.height, 10.0);
        assert!(!d.failed);

        // Hollowing.cpp:162-167 — radius/height compared with is_approx (EPSILON=1e-4)
        let mut d2 = d.clone();
        d2.radius = 5.00005;
        assert_eq!(d, d2);
        d2.radius = 5.1;
        assert_ne!(d, d2);
    }

    #[test]
    fn swap_normals_swaps_first_and_last_index() {
        // Hollowing.hpp:100-104
        let mut its = indexed_triangle_set::default();
        its.indices.push(Vec3i::new(1, 2, 3));
        swap_normals(&mut its);
        assert_eq!(its.indices[0], Vec3i::new(3, 2, 1));
    }

    #[test]
    fn drain_hole_is_inside() {
        // Cylinder at origin pointing +Z, r=5, h=10 (Hollowing.cpp:169-181).
        let d = DrainHole::default();
        assert!(d.is_inside(&Vec3f::new(0.0, 0.0, 5.0)));
        assert!(d.is_inside(&Vec3f::new(4.9, 0.0, 5.0)));
        assert!(!d.is_inside(&Vec3f::new(5.1, 0.0, 5.0)));
        // Behind the base plane (dist < EPSILON):
        assert!(!d.is_inside(&Vec3f::new(0.0, 0.0, -1.0)));
        // Past the top:
        assert!(!d.is_inside(&Vec3f::new(0.0, 0.0, 11.0)));
    }

    #[test]
    fn drain_hole_get_intersections_axis_ray() {
        // Ray along the cylinder axis through both bases (Hollowing.cpp:187-278).
        let d = DrainHole::default();
        let mut out = [(0.0f32, Vec3d::zeros()); 2];
        let hit = d.get_intersections(
            &Vec3f::new(0.0, 0.0, -5.0),
            &Vec3f::new(0.0, 0.0, 1.0),
            &mut out,
        );
        assert!(hit);
        // Sorted by parameter (Hollowing.cpp:274-275); base at z ~ -EPSILON,
        // top base at z = 10 -> parameters ~5 and ~15.
        assert!((out[0].0 - 5.0).abs() < 1e-3);
        assert!((out[1].0 - 15.0).abs() < 1e-3);
        // Normals point inside the hole (Hollowing.cpp:223).
        assert!((out[0].1 - Vec3d::new(0.0, 0.0, 1.0)).norm() < 1e-6);
        assert!((out[1].1 - Vec3d::new(0.0, 0.0, -1.0)).norm() < 1e-6);
    }

    #[test]
    fn drain_hole_get_intersections_wall_ray() {
        // Ray perpendicular to the axis through the middle (wall hits,
        // Hollowing.cpp:236-266).
        let d = DrainHole::default();
        let mut out = [(0.0f32, Vec3d::zeros()); 2];
        let hit = d.get_intersections(
            &Vec3f::new(-10.0, 0.0, 5.0),
            &Vec3f::new(1.0, 0.0, 0.0),
            &mut out,
        );
        assert!(hit);
        assert!((out[0].0 - 5.0).abs() < 1e-3); // x = -5 wall
        assert!((out[1].0 - 15.0).abs() < 1e-3); // x = +5 wall
        // Wall normals point towards the axis.
        assert!((out[0].1 - Vec3d::new(1.0, 0.0, 0.0)).norm() < 1e-5);
        assert!((out[1].1 - Vec3d::new(-1.0, 0.0, 0.0)).norm() < 1e-5);
    }

    #[test]
    fn drain_hole_to_mesh_rotated_and_translated() {
        // Hollowing.cpp:150-160 — cylinder rotated from +Z to `normal`, then
        // offset by pos.
        let d = DrainHole::new(Vec3f::new(1.0, 2.0, 3.0), Vec3f::new(1.0, 0.0, 0.0), 2.0, 4.0, false);
        let m = d.to_mesh();
        assert_eq!(m.vertices.len(), 2 * DrainHole::STEPS);
        // The +Z axis maps to +X: all vertices lie within x in [3-eps, 7+eps]
        // around pos.x... cylinder() spans z in [0, -h]? Verify via bounds.
        let (mut min_x, mut max_x) = (f32::INFINITY, f32::NEG_INFINITY);
        for v in &m.vertices {
            min_x = min_x.min(v.x);
            max_x = max_x.max(v.x);
        }
        assert!((max_x - min_x - 4.0).abs() < 1e-4); // height along +X
    }

    #[test]
    fn generate_interior_errors_on_native_openvdb() {
        // The OpenVDB grid pipeline is native-blocked: generate_interior must
        // surface an explicit error, never fake data.
        let mut its = indexed_triangle_set::default();
        its.vertices.push(Vec3f::new(0.0, 0.0, 0.0));
        its.vertices.push(Vec3f::new(1.0, 0.0, 0.0));
        its.vertices.push(Vec3f::new(0.0, 1.0, 0.0));
        its.indices.push(Vec3i::new(0, 1, 2));
        let res = generate_interior(&its, &HollowingConfig::default(), &JobController::default());
        assert!(res.is_err());
    }

    #[test]
    fn interior_defaults() {
        // Hollowing.cpp:30-34
        let i = Interior::default();
        assert_eq!(i.closing_distance, 0.0);
        assert_eq!(i.thickness, 0.0);
        assert_eq!(i.voxel_scale, 1.0);
        assert_eq!(i.nb_in, 3.0);
        assert_eq!(i.nb_out, 3.0);
        assert!(i.gridptr.is_none());
    }
}
