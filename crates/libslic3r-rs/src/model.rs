//! Port of `Model.hpp` / `Model.cpp` from Bambu Studio's libslic3r (pin 56a57d0).
//!
//! The C++ classes `Model`, `ModelObject`, `ModelVolume`, `ModelInstance`, `ModelMaterial`,
//! `ModelConfigObject`, `ModelWipeTower`, `LayerHeightProfile`, `FacetsAnnotation` and the cut
//! connector structs, with the functions of Model.cpp in its order.
//!
//! Two layers live side by side:
//! - the members the loaders (`stl.rs`, `format/obj.rs`, `format/amf.rs`, `format/three_mf.rs`)
//!   and `slicing_adaptive.rs` have always filled and read: `ModelObject::mesh` (one merged mesh),
//!   `ModelInstance::{position, rotation_z, scale}`, `Model::metadata` / `file_path`. They keep
//!   their behaviour (nothing on the slicing path reads `crate::model`);
//! - the C++ members: `ModelObject::volumes` with their meshes, transformations, configs and
//!   paint, `ModelInstance` transformations, object ids, materials. Every Model.cpp function
//!   ported here works on these.
//!
//! FIDELITY-NOTE (back pointers): `ModelVolume::object`, `ModelInstance::object`,
//! `ModelObject::m_model` and `ModelMaterial::m_model` are raw parent pointers in C++. Rust owns
//! the tree by value, so a function that reads the parent takes it as an argument, and one that
//! changes the parent's containers (`ModelVolume::split`, `ModelObject::split`) is called with the
//! parent. The pointers themselves are kept as handles: the `ObjectID` of the parent (`None` for
//! `nullptr`), set where the C++ sets the pointer (the constructors that take the parent,
//! `set_model`, `set_model_object`, `assign_copy`, `update_links_bottom_up_recursive`). A pointer
//! stays valid when its parent takes a new ID; a handle is moved along with the ID
//! (`set_new_unique_id` / `assign_new_unique_ids_recursive` of the parent re-point the children
//! that held the old one).
//!
//! FIDELITY-NOTE (meshes): a C++ `TriangleMesh` is `its` plus `TriangleMeshStats`; the crate's
//! `triangle_mesh::TriangleMesh` is the earlier f64 mesh without stats. A volume holds its mesh
//! as [`ModelMesh`], which is the C++ class as Model.cpp uses it (TriangleMesh.cpp bodies).
//!
//! FIDELITY-NOTE (Eigen arithmetic): the C++ build is clang with `-ffp-contract=on` and Eigen
//! 5.0.1; affine products and `Transform3d * Vec3d` are evaluated here in the order that build
//! evaluates them (`eigen_*` helpers below, checked against the native vectors of
//! `tests/model_native_parity.rs`).

use crate::bounding_box::BoundingBoxf3;
use crate::geometry::geometry::{Axis, Transform3d, Transformation, Vec2d, Vec3d};
use crate::geometry::{BoundingBox3F, Point3F, Transform3D};
use crate::normal_utils::indexed_triangle_set;
use crate::object_id::{CutObjectBase, ObjectBase, ObjectID, ObjectWithTimestamp, Timestamp};
use crate::print_config::config_def::DynamicPrintConfig;
use crate::triangle_mesh::{TriangleMesh, TriangleMeshStats, Vec3f, Vec3i};
use crate::triangle_selector::EnforcerBlockerType;
use crate::Polygon;
use nalgebra::Matrix3;
use std::cell::Cell;
use std::collections::{BTreeMap, HashMap};
use std::path::PathBuf;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

type Matrix3d = Matrix3<f64>;
/// The vector type of `bounding_box::BoundingBoxf3`.
type BbVec3 = crate::geometry::Vec3d;

#[inline]
fn to_bb(v: &Vec3d) -> BbVec3 {
    BbVec3::new(v.x, v.y, v.z)
}

#[inline]
fn from_bb(v: &BbVec3) -> Vec3d {
    Vec3d::new(v.x, v.y, v.z)
}

/// Model.cpp:58-61
pub const CONST_FILAMENTS: [&str; 33] = [
    "", "4", "8", "0C", "1C", "2C", "3C", "4C", "5C", "6C", "7C", "8C", "9C", "AC", "BC", "CC",
    "DC", // 16
    "EC", "0FC", "1FC", "2FC", "3FC", "4FC", "5FC", "6FC", "7FC", "8FC", "9FC", "AFC", "BFC",
    "CFC", "DFC", "EFC", // 32
]; //      1                         5                                 10                                 15    16

/// libslic3r.h:52
const EPSILON: f64 = 1e-4;
/// libslic3r.h:58
const SCALING_FACTOR: f64 = 0.00001;

// =============================================================================================
// Eigen arithmetic as the C++ build evaluates it
// =============================================================================================

// The Eigen operations shared with Geometry.cpp's port live in `geometry::eigen` (checked against
// the pin by tests/geometry_transformation_native_parity.rs); these are the names Model.cpp's port
// calls them by.

/// The last row of an Eigen lazy product of a 3-row left-hand side, `coeff()`: `e0 + (e1 + e2)`.
#[inline]
fn eigen_tree_dot(a0: f64, a1: f64, a2: f64, x: f64, y: f64, z: f64) -> f64 {
    crate::geometry::geometry::eigen::coeff_dot3(a0, a1, a2, x, y, z)
}

/// One row of an Eigen lazy product evaluated in a packet, `fma(m2, z, fma(m1, y, m0 * x))`.
#[inline]
fn eigen_row_dot(m: &Transform3d, row: usize, x: f64, y: f64, z: f64) -> f64 {
    crate::geometry::geometry::eigen::packet_dot3(m[(row, 0)], m[(row, 1)], m[(row, 2)], x, y, z)
}

/// Eigen `Transform3d * Vec3d` of an affine transform ([`crate::geometry::geometry::eigen::transform_mul_vec`]).
#[inline]
pub fn eigen_transform_point(t: &Transform3d, v: &Vec3d) -> Vec3d {
    crate::geometry::geometry::eigen::transform_mul_vec(t, v)
}

/// Eigen `Matrix3d * Vec3d` ([`crate::geometry::geometry::eigen::mat3_mul_vec`]).
#[inline]
pub fn eigen_matrix3_mul_vec(m: &Matrix3d, v: &Vec3d) -> Vec3d {
    crate::geometry::geometry::eigen::mat3_mul_vec(m, v)
}

/// Eigen `Transform3d * Transform3d` of two affine transforms
/// ([`crate::geometry::geometry::eigen::transform_mul`]).
#[inline]
pub fn eigen_mul_affine(a: &Transform3d, b: &Transform3d) -> Transform3d {
    crate::geometry::geometry::eigen::transform_mul(a, b)
}

/// Eigen `Matrix3d * Matrix3d` ([`crate::geometry::geometry::eigen::mat3_mul`]).
#[inline]
pub fn eigen_matrix3_mul(a: &Matrix3d, b: &Matrix3d) -> Matrix3d {
    crate::geometry::geometry::eigen::mat3_mul(a, b)
}

/// The linear part of an affine transform.
#[inline]
fn linear_of(t: &Transform3d) -> Matrix3d {
    t.fixed_view::<3, 3>(0, 0).into_owned()
}

/// An affine transform of a linear part, `Transform3d(Matrix3d)`.
#[inline]
fn affine_of(m: &Matrix3d) -> Transform3d {
    let mut t = Transform3d::identity();
    t.fixed_view_mut::<3, 3>(0, 0).copy_from(m);
    t
}

/// Eigen `Matrix3d::determinant()` ([`crate::geometry::geometry::eigen::determinant3`]).
#[inline]
pub fn eigen_determinant3(m: &Matrix3d) -> f64 {
    crate::geometry::geometry::eigen::determinant3(m)
}

/// Eigen `Transform3d::inverse()` of an affine transform
/// ([`crate::geometry::geometry::eigen::transform_inverse`]).
#[inline]
pub fn eigen_inverse_affine(t: &Transform3d) -> Transform3d {
    crate::geometry::geometry::eigen::transform_inverse(t)
}

/// BoundingBox.cpp:265-294 `BoundingBoxf3::transformed(const Transform3d &matrix)`: the eight
/// corners through `matrix * src_vertices.colwise().homogeneous()`, which Eigen evaluates as the
/// linear product followed by the translation (Eigen/src/Geometry/Homogeneous.h).
pub fn eigen_bbox_transformed(bb: &BoundingBoxf3, matrix: &Transform3d) -> BoundingBoxf3 {
    let (mn, mx) = (from_bb(&bb.min), from_bb(&bb.max));
    let src = [
        Vec3d::new(mn.x, mn.y, mn.z),
        Vec3d::new(mx.x, mn.y, mn.z),
        Vec3d::new(mx.x, mx.y, mn.z),
        Vec3d::new(mn.x, mx.y, mn.z),
        Vec3d::new(mn.x, mn.y, mx.z),
        Vec3d::new(mx.x, mn.y, mx.z),
        Vec3d::new(mx.x, mx.y, mx.z),
        Vec3d::new(mn.x, mx.y, mx.z),
    ];
    // dst = linear * src (a lazy product: rows 0 and 1 a packet, row 2 `coeff()`), then
    // dst += translation (Homogeneous.h homogeneous_left_product_impl::evalTo)
    let dst: Vec<Vec3d> = src
        .iter()
        .map(|v| {
            Vec3d::new(
                eigen_row_dot(matrix, 0, v.x, v.y, v.z) + matrix[(0, 3)],
                eigen_row_dot(matrix, 1, v.x, v.y, v.z) + matrix[(1, 3)],
                eigen_tree_dot(matrix[(2, 0)], matrix[(2, 1)], matrix[(2, 2)], v.x, v.y, v.z) + matrix[(2, 3)],
            )
        })
        .collect();
    let mut v_min = dst[0];
    let mut v_max = v_min;
    for d in dst.iter().skip(1) {
        for j in 0..3 {
            v_min[j] = v_min[j].min(d[j]);
            v_max[j] = v_max[j].max(d[j]);
        }
    }
    BoundingBoxf3::new_from_points(to_bb(&v_min), to_bb(&v_max))
}

/// `std::min` of doubles: `(b < a) ? b : a`.
#[inline]
fn std_min(a: f64, b: f64) -> f64 {
    if b < a {
        b
    } else {
        a
    }
}

/// `std::max` of doubles: `(a < b) ? b : a`.
#[inline]
fn std_max(a: f64, b: f64) -> f64 {
    if a < b {
        b
    } else {
        a
    }
}

/// BoundingBox.cpp:137-148 `BoundingBox3Base<Vec3d>::merge(const Vec3d &point)`.
fn bbox_merge_point(bb: &mut BoundingBoxf3, point: &Vec3d) {
    if bb.defined {
        bb.min = BbVec3::new(std_min(bb.min.x, point.x), std_min(bb.min.y, point.y), std_min(bb.min.z, point.z));
        bb.max = BbVec3::new(std_max(bb.max.x, point.x), std_max(bb.max.y, point.y), std_max(bb.max.z, point.z));
    } else {
        bb.min = to_bb(point);
        bb.max = to_bb(point);
        bb.defined = true;
    }
}

/// BoundingBox.cpp:159-173 `BoundingBox3Base<Vec3d>::merge(const BoundingBox3Base &bb)`.
fn bbox_merge(bb: &mut BoundingBoxf3, other: &BoundingBoxf3) {
    if other.defined {
        if bb.defined {
            bb.min = BbVec3::new(std_min(bb.min.x, other.min.x), std_min(bb.min.y, other.min.y), std_min(bb.min.z, other.min.z));
            bb.max = BbVec3::new(std_max(bb.max.x, other.max.x), std_max(bb.max.y, other.max.y), std_max(bb.max.z, other.max.z));
        } else {
            bb.min = other.min;
            bb.max = other.max;
            bb.defined = true;
        }
    }
}

/// `Vec3d::squaredNorm()` as the C++ build evaluates it, `fma(z, z, x * x + y * y)`.
#[inline]
fn eigen_squared_norm3(v: &Vec3d) -> f64 {
    v.z.mul_add(v.z, v.x * v.x + v.y * v.y)
}

/// Eigen `a.isApprox(b)` for `Vec3d` with the default precision (1e-12): `(a - b).squaredNorm()
/// <= prec^2 * min(a.squaredNorm(), b.squaredNorm())`.
fn eigen_is_approx_vec3(a: &Vec3d, b: &Vec3d) -> bool {
    let prec = 1e-12;
    let d = a - b;
    let sq = |v: &Vec3d| v.x * v.x + v.y * v.y + v.z * v.z;
    sq(&d) <= prec * prec * sq(a).min(sq(b))
}

/// Eigen `a.isApprox(b)` for a `Transform3d` (its 4x4 matrix) with the default precision.
fn eigen_is_approx_mat4(a: &Transform3d, b: &Transform3d) -> bool {
    let prec = 1e-12;
    let (mut dd, mut aa, mut bb) = (0.0, 0.0, 0.0);
    for c in 0..4 {
        for r in 0..4 {
            let d = a[(r, c)] - b[(r, c)];
            dd += d * d;
            aa += a[(r, c)] * a[(r, c)];
            bb += b[(r, c)] * b[(r, c)];
        }
    }
    dd <= prec * prec * aa.min(bb)
}

/// Eigen `Quaterniond(AngleAxisd(angle, axis)).toRotationMatrix()`
/// ([`crate::geometry::geometry::eigen::Quaterniond`]).
pub fn eigen_angle_axis_rotation_matrix(angle: f64, axis: &Vec3d) -> Matrix3d {
    crate::geometry::geometry::eigen::Quaterniond::from_angle_axis(angle, axis).to_rotation_matrix()
}

/// `scale_(v)` then `coord_t(...)`: divide by SCALING_FACTOR and truncate.
#[inline]
fn coord_scale(v: f64) -> crate::Coord {
    (v / SCALING_FACTOR) as crate::Coord
}

// =============================================================================================
// Model.hpp:244-342 — cut enums and connectors, ModelVolumeType
// =============================================================================================

/// Model.hpp:244-247
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum CutMode {
    CutPlanar,
    CutTongueAndGroove,
}

/// Model.hpp:249-255
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(i32)]
pub enum CutConnectorType {
    Plug,
    Dowel,
    Snap,
    Thread,
    Undef,
}

/// Model.hpp:257-262
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(i32)]
pub enum CutConnectorStyle {
    Prizm,
    Frustum,
    Undef,
}

/// Model.hpp:264-271
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(i32)]
pub enum CutConnectorShape {
    Triangle,
    Square,
    Hexagon,
    Circle,
    Undef,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CutConnectorParas {
    /// Model.hpp:274
    pub snap_space_proportion: f32,
    /// Model.hpp:275
    pub snap_bulge_proportion: f32,
}

impl Default for CutConnectorParas {
    fn default() -> Self {
        Self {
            snap_space_proportion: 0.3,
            snap_bulge_proportion: 0.15,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CutConnectorAttributes {
    /// Model.hpp:280 — CutConnectorType type{CutConnectorType::Plug};
    pub type_: CutConnectorType,
    /// Model.hpp:281 — CutConnectorStyle style{CutConnectorStyle::Prizm};
    pub style: CutConnectorStyle,
    /// Model.hpp:282 — CutConnectorShape shape{CutConnectorShape::Circle};
    pub shape: CutConnectorShape,
}

impl CutConnectorAttributes {
    /// Model.hpp:286
    pub fn with(type_: CutConnectorType, style: CutConnectorStyle, shape: CutConnectorShape) -> Self {
        Self { type_, style, shape }
    }
}

impl Default for CutConnectorAttributes {
    fn default() -> Self {
        Self {
            type_: CutConnectorType::Plug,
            style: CutConnectorStyle::Prizm,
            shape: CutConnectorShape::Circle,
        }
    }
}

impl PartialOrd for CutConnectorAttributes {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

/// Model.hpp:294-298 `operator<`
impl Ord for CutConnectorAttributes {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.type_
            .cmp(&other.type_)
            .then(self.style.cmp(&other.style))
            .then(self.shape.cmp(&other.shape))
    }
}

/// Model.hpp:303-326 `struct CutConnector`
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CutConnector {
    pub pos: Vec3d,
    pub rotation_m: Transform3d,
    pub radius: f32,
    pub height: f32,
    /// [0.f : 1.f]
    pub radius_tolerance: f32,
    /// [0.f : 1.f]
    pub height_tolerance: f32,
    pub attribs: CutConnectorAttributes,
    pub paras: CutConnectorParas,
}

impl Default for CutConnector {
    /// Model.hpp:313
    fn default() -> Self {
        Self {
            pos: Vec3d::zeros(),
            rotation_m: Transform3d::identity(),
            radius: 5.0,
            height: 10.0,
            radius_tolerance: 0.0,
            height_tolerance: 0.1,
            attribs: CutConnectorAttributes::default(),
            paras: CutConnectorParas::default(),
        }
    }
}

impl CutConnector {
    /// Model.hpp:315-317
    pub fn new(p: Vec3d, rot: Transform3d, r: f32, h: f32, rt: f32, ht: f32, attributes: CutConnectorAttributes) -> Self {
        Self {
            pos: p,
            rotation_m: rot,
            radius: r,
            height: h,
            radius_tolerance: rt,
            height_tolerance: ht,
            attribs: attributes,
            paras: CutConnectorParas::default(),
        }
    }
}

/// Model.hpp:328 `using CutConnectors = std::vector<CutConnector>;`
pub type CutConnectors = Vec<CutConnector>;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(i32)]
pub enum ModelVolumeType {
    /// Model.hpp:332
    Invalid = -1,
    /// Model.hpp:333
    ModelPart = 0,
    /// Model.hpp:334
    NegativeVolume,
    /// Model.hpp:335
    ParameterModifier,
    /// Model.hpp:336
    SupportBlocker,
    /// Model.hpp:337
    SupportEnforcer,
}

impl ModelVolumeType {
    pub fn type_from_string(s: &str) -> ModelVolumeType {
        ModelVolume::type_from_string(s)
    }

    pub fn type_to_string(t: ModelVolumeType) -> &'static str {
        ModelVolume::type_to_string(t)
    }
}

/// Model.hpp:340 `enum class ModelObjectCutAttribute`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum ModelObjectCutAttribute {
    KeepUpper,
    KeepLower,
    FlipUpper,
    FlipLower,
    PlaceOnCutUpper,
    PlaceOnCutLower,
    CreateDowels,
    CutToParts,
    InvalidateCutInfo,
}

/// Model.hpp:341 `using ModelObjectCutAttributes = enum_bitmask<ModelObjectCutAttribute>;`
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ModelObjectCutAttributes(pub u32);

impl ModelObjectCutAttributes {
    /// enum_bitmask.hpp `bool has(option_type o) const`
    pub fn has(&self, o: ModelObjectCutAttribute) -> bool {
        self.0 & (1u32 << (o as u32)) != 0
    }
    /// enum_bitmask.hpp `operator|`
    pub fn with(self, o: ModelObjectCutAttribute) -> Self {
        Self(self.0 | (1u32 << (o as u32)))
    }
}

/// Model.hpp:718-720
pub const MAX_PHYSICAL_FILAMENT_COUNT: usize = 32;
pub const MAX_MIXED_FILAMENT_COUNT: usize = 64;
pub const MAX_FILAMENT_COUNT: usize = MAX_PHYSICAL_FILAMENT_COUNT + MAX_MIXED_FILAMENT_COUNT;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum ConversionType {
    ConvToInch,
    ConvFromInch,
    ConvToMeter,
    ConvFromMeter,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum En3mfType {
    FromBBS,
    FromPrusa,
    FromOther,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ModelInstanceEPrintVolumeState {
    /// Model.hpp:1447
    ModelInstancePvsInside,
    /// Model.hpp:1448
    ModelInstancePvsLimited,
    /// Model.hpp:1449
    ModelInstancePvsPartlyOutside,
    /// Model.hpp:1450
    ModelInstancePvsFullyOutside,
    /// Model.hpp:1451
    ModelInstanceNumBedStates,
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct ExtruderParams {
    /// Model.hpp:1636
    pub material_name: String,
    /// Model.hpp:1638
    pub bed_temp: i32,
    /// Model.hpp:1639
    pub heat_end_temp: f64,
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct GlobalSpeedMap {
    /// Model.hpp:1644
    pub perimeter_speed: f64,
    /// Model.hpp:1645
    pub external_perimeter_speed: f64,
    /// Model.hpp:1646
    pub infill_speed: f64,
    /// Model.hpp:1647
    pub solid_infill_speed: f64,
    /// Model.hpp:1648
    pub top_solid_infill_speed: f64,
    /// Model.hpp:1649
    pub support_speed: f64,
    /// Model.hpp:1650
    pub small_perimeter_speed: f64,
    /// Model.hpp:1651
    pub max_speed: f64,
    /// Model.hpp:1652
    pub bed_poly: Polygon,
}

/// Model.cpp:63 `std::map<size_t, ExtruderParams> Model::extruderParamsMap = { {0,{"",0,0}}};`
pub fn extruder_params_map_default() -> std::collections::BTreeMap<usize, ExtruderParams> {
    let mut m = std::collections::BTreeMap::new();
    m.insert(
        0usize,
        ExtruderParams {
            material_name: String::new(),
            bed_temp: 0,
            heat_end_temp: 0.0,
        },
    );
    m
}

/// Model.cpp:64 `GlobalSpeedMap Model::printSpeedMap{};`
pub fn print_speed_map_default() -> GlobalSpeedMap {
    GlobalSpeedMap::default()
}

/// Model.hpp:1841 `static Polygon getBedPolygon() { return Model::printSpeedMap.bed_poly; }`
pub fn get_bed_polygon(print_speed_map: &GlobalSpeedMap) -> Polygon {
    print_speed_map.bed_poly.clone()
}

/// Model.hpp:2006
pub const SINKING_Z_THRESHOLD: f32 = -0.001;
/// Model.hpp:2007
pub const SINKING_MIN_Z_THRESHOLD: f64 = 0.05;

// =============================================================================================
// PrintConfig.hpp:2098-2172 — ModelConfig; Model.hpp:73-103 — ModelConfigObject
// =============================================================================================

/// PrintConfig.cpp:11448 `std::atomic<uint64_t> ModelConfig::s_last_timestamp { 1 };`
static MODEL_CONFIG_S_LAST_TIMESTAMP: AtomicU64 = AtomicU64::new(1);

/// PrintConfig.hpp:2098-2172 `class ModelConfig`: a `DynamicPrintConfig` with a timestamp.
#[derive(Clone)]
pub struct ModelConfig {
    /// PrintConfig.hpp:2166 `uint64_t m_timestamp { 1 };`
    m_timestamp: u64,
    /// PrintConfig.hpp:2167
    m_data: DynamicPrintConfig,
}

impl Default for ModelConfig {
    fn default() -> Self {
        Self { m_timestamp: 1, m_data: DynamicPrintConfig::new() }
    }
}

impl std::fmt::Debug for ModelConfig {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ModelConfig").field("timestamp", &self.m_timestamp).field("keys", &self.keys()).finish()
    }
}

impl ModelConfig {
    /// PrintConfig.hpp:2103
    pub fn reset(&mut self) {
        self.m_data.clear();
        self.touch();
    }

    /// PrintConfig.hpp:2105-2110 `assign_config(const ModelConfig &rhs)`
    pub fn assign_config(&mut self, rhs: &ModelConfig) {
        if self.m_timestamp != rhs.m_timestamp {
            self.m_data = rhs.m_data.clone();
            self.m_timestamp = rhs.m_timestamp;
        }
    }

    /// PrintConfig.hpp:2122 `assign_config(const DynamicPrintConfig &rhs)`
    pub fn assign_config_data(&mut self, rhs: &DynamicPrintConfig) {
        if self.m_data.operator_ne(rhs).unwrap_or(true) {
            self.m_data = rhs.clone();
            self.touch();
        }
    }

    /// PrintConfig.hpp:2124-2125 `apply(const ModelConfig &other, bool ignore_nonexistent)`:
    /// `m_data.apply_only(other, other.keys(), ignore_nonexistent); touch();`
    pub fn apply(&mut self, other: &ModelConfig, ignore_nonexistent: bool) {
        use crate::config::ConfigBase;
        let keys = ConfigBase::keys(&other.m_data);
        let _ = self.m_data.apply_only(&other.m_data, &keys, ignore_nonexistent);
        self.touch();
    }

    /// PrintConfig.hpp:2128
    pub fn set_key_value(&mut self, opt_key: &str, opt: Box<dyn crate::config::ConfigOption>) -> bool {
        let out = self.m_data.set_key_value(opt_key, opt);
        self.touch();
        out
    }

    /// PrintConfig.hpp:2129-2130 `template<typename T> void set(const std::string &opt_key, T value)`
    /// for `T = int`: `m_data.set(opt_key, value, true); touch();`
    pub fn set(&mut self, opt_key: &str, value: i32) {
        use crate::config::ConfigBase;
        let _ = self.m_data.set_int(opt_key, value, true);
        self.touch();
    }

    /// PrintConfig.hpp:2126-2127 `set_deserialize(opt_key, str, substitution_context, append)`:
    /// `m_data.set_deserialize(...); touch();` (no touch when it throws).
    pub fn set_deserialize(
        &mut self,
        opt_key: &str,
        str: &str,
        substitution_context: &mut crate::config::ConfigSubstitutionContext,
        append: bool,
    ) -> Result<(), crate::config::ConfigurationError> {
        use crate::config::ConfigBase;
        self.m_data.set_deserialize(opt_key, str, substitution_context, append)?;
        self.touch();
        Ok(())
    }

    /// PrintConfig.hpp:2133
    pub fn erase(&mut self, opt_key: &str) -> bool {
        let out = self.m_data.erase(opt_key);
        if out {
            self.touch();
        }
        out
    }

    /// PrintConfig.hpp:2138
    pub fn get(&self) -> &DynamicPrintConfig {
        &self.m_data
    }
    /// PrintConfig.hpp:2139
    pub fn empty(&self) -> bool {
        self.m_data.empty()
    }
    /// PrintConfig.hpp:2140
    pub fn size(&self) -> usize {
        self.m_data.size()
    }
    /// PrintConfig.hpp:2143
    pub fn keys(&self) -> Vec<String> {
        crate::config::ConfigBase::keys(&self.m_data)
    }
    /// PrintConfig.hpp:2144
    pub fn has(&self, opt_key: &str) -> bool {
        self.option(opt_key).is_some()
    }
    /// PrintConfig.hpp:2145
    pub fn option(&self, opt_key: &str) -> Option<&dyn crate::config::ConfigOption> {
        use crate::config::ConfigOptionResolver;
        self.m_data.optptr(opt_key)
    }
    /// PrintConfig.hpp:2146
    pub fn opt_int(&self, opt_key: &str) -> i32 {
        self.m_data.opt_int(opt_key)
    }
    /// PrintConfig.hpp:2147
    pub fn extruder(&self) -> i32 {
        self.opt_int("extruder")
    }
    /// PrintConfig.hpp:2155
    pub fn timestamp(&self) -> u64 {
        self.m_timestamp
    }
    /// PrintConfig.hpp:2156
    pub fn timestamp_matches(&self, rhs: &ModelConfig) -> bool {
        self.m_timestamp == rhs.m_timestamp
    }
    /// PrintConfig.hpp:2157
    pub fn touch(&mut self) {
        self.m_timestamp = MODEL_CONFIG_S_LAST_TIMESTAMP.fetch_add(1, Ordering::SeqCst) + 1;
    }
    /// PrintConfig.hpp:2158-2160
    pub fn operator_eq(&self, other: &ModelConfig) -> bool {
        self.m_data.operator_eq(&other.m_data).unwrap_or(false)
    }
}

/// `getInt()` of an option, 0 for an option without an integer value.
fn opt_get_int(opt: &dyn crate::config::ConfigOption) -> i32 {
    opt.get_int().unwrap_or(0)
}

/// Model.hpp:73-103 `class ModelConfigObject : public ObjectBase, public ModelConfig`
#[derive(Clone, Debug)]
pub struct ModelConfigObject {
    base: ObjectBase,
    config: ModelConfig,
}

impl Default for ModelConfigObject {
    /// Model.hpp:84 — default constructor assigns a unique ID.
    fn default() -> Self {
        Self::new()
    }
}

impl std::ops::Deref for ModelConfigObject {
    type Target = ModelConfig;
    fn deref(&self) -> &ModelConfig {
        &self.config
    }
}

impl std::ops::DerefMut for ModelConfigObject {
    fn deref_mut(&mut self) -> &mut ModelConfig {
        &mut self.config
    }
}

impl ModelConfigObject {
    /// Model.hpp:84 `explicit ModelConfigObject() = default;`
    pub fn new() -> Self {
        Self { base: ObjectBase::new(), config: ModelConfig::default() }
    }
    /// Model.hpp:87 `explicit ModelConfigObject(int) : ObjectBase(-1) {}`
    pub fn new_invalid() -> Self {
        Self { base: ObjectBase::new_invalid(), config: ModelConfig::default() }
    }
    pub fn id(&self) -> ObjectID {
        self.base.id()
    }
    pub fn set_new_unique_id(&mut self) {
        self.base.set_new_unique_id();
    }
    /// Model.hpp:93
    pub fn timestamp(&self) -> Timestamp {
        self.config.timestamp()
    }
    /// Model.hpp:94
    pub fn object_id_and_timestamp_match(&self, rhs: &ModelConfigObject) -> bool {
        self.id() == rhs.id() && self.timestamp() == rhs.timestamp()
    }
}

// =============================================================================================
// Model.hpp:162-203 — ModelMaterial
// =============================================================================================

/// Model.hpp:120-124
pub type TModelMaterialId = String;
pub type TModelMaterialAttributes = BTreeMap<String, String>;
pub type ModelMaterialMap = BTreeMap<TModelMaterialId, ModelMaterial>;

/// Model.hpp:162-203 `class ModelMaterial final : public ObjectBase`
#[derive(Clone, Debug)]
pub struct ModelMaterial {
    base: ObjectBase,
    /// Model.hpp:166
    pub attributes: TModelMaterialAttributes,
    /// Model.hpp:168
    pub config: ModelConfigObject,
    /// Model.hpp:176 `Model *m_model`: the parent model, held as its ID (FIDELITY-NOTE (back
    /// pointers) at the top of this file); `None` is the C++ `nullptr`.
    m_model: Option<ObjectID>,
}

impl ModelMaterial {
    /// Model.hpp:181 `ModelMaterial(Model *model)`: a new ID for the material and its config.
    /// The parent is set by the model that adds it (`Model::add_material`).
    fn new() -> Self {
        let base = ObjectBase::new();
        Self { base, attributes: BTreeMap::new(), config: ModelConfigObject::new(), m_model: None }
    }
    pub fn id(&self) -> ObjectID {
        self.base.id()
    }
    /// Model.hpp:170 `Model* get_model() const { return m_model; }`: the ID of the parent model.
    pub fn get_model(&self) -> Option<ObjectID> {
        self.m_model
    }
    /// Model.hpp:184 `void set_model(Model *model) { m_model = model; }`: `model` is the ID of
    /// the parent model, `None` for `nullptr`.
    pub fn set_model(&mut self, model: Option<ObjectID>) {
        self.m_model = model;
    }
    /// Model.hpp:171-172
    pub fn apply(&mut self, attributes: &TModelMaterialAttributes) {
        for (k, v) in attributes {
            self.attributes.entry(k.clone()).or_insert_with(|| v.clone());
        }
    }
    /// Model.hpp:185
    pub fn set_new_unique_id(&mut self) {
        self.base.set_new_unique_id();
        self.config.set_new_unique_id();
    }
    /// ObjectBase::assign_new_unique_ids_recursive (not overridden by ModelMaterial)
    fn assign_new_unique_ids_recursive(&mut self) {
        self.base.set_new_unique_id();
    }
}

/// Model.cpp:787-794 `Model::add_material(t_model_material_id material_id)` on the material map.
fn materials_add(materials: &mut ModelMaterialMap, material_id: &str) {
    // assert(!material_id.is_empty()): compiled out in the reference build (-DNDEBUG)
    if !materials.contains_key(material_id) {
        materials.insert(material_id.to_string(), ModelMaterial::new());
    }
}

/// Model.cpp:796-807 `Model::add_material(material_id, const ModelMaterial &other)` on the map.
fn materials_add_copy(materials: &mut ModelMaterialMap, material_id: &str, other: &ModelMaterial) {
    // assert(!material_id.is_empty()): compiled out in the reference build (-DNDEBUG)
    // delete existing material if any; set new material (copy, IDs included)
    materials.insert(material_id.to_string(), other.clone());
}

// =============================================================================================
// Model.hpp:205-242 — LayerHeightProfile
// =============================================================================================

/// Model.hpp:205-242 `class LayerHeightProfile final : public ObjectWithTimestamp`
#[derive(Clone, Debug)]
pub struct LayerHeightProfile {
    base: ObjectWithTimestamp,
    m_data: Vec<f64>,
}

impl LayerHeightProfile {
    /// Model.hpp:225 — default constructor assigns a unique ID.
    pub fn new() -> Self {
        Self { base: ObjectWithTimestamp::new(), m_data: Vec::new() }
    }
    /// Model.hpp:228
    pub fn new_invalid() -> Self {
        Self { base: ObjectWithTimestamp::new_invalid(), m_data: Vec::new() }
    }
    pub fn id(&self) -> ObjectID {
        self.base.id()
    }
    pub fn timestamp(&self) -> Timestamp {
        self.base.timestamp()
    }
    pub fn set_new_unique_id(&mut self) {
        self.base.set_new_unique_id();
    }
    /// Model.hpp:208
    pub fn assign(&mut self, rhs: &LayerHeightProfile) {
        if !self.base.timestamp_matches(&rhs.base) {
            self.m_data = rhs.m_data.clone();
            self.base.copy_timestamp(&rhs.base);
        }
    }
    /// Model.hpp:211
    pub fn get(&self) -> Vec<f64> {
        self.m_data.clone()
    }
    /// Model.hpp:212
    pub fn empty(&self) -> bool {
        self.m_data.is_empty()
    }
    /// Model.hpp:213
    pub fn set(&mut self, data: &[f64]) {
        if self.m_data != data {
            self.m_data = data.to_vec();
            self.base.touch();
        }
    }
    /// Model.hpp:215
    pub fn clear(&mut self) {
        self.m_data.clear();
        self.base.touch();
    }
}

// =============================================================================================
// TriangleMesh as Model.cpp uses it
// =============================================================================================

/// TriangleMesh.hpp:95-170 `class TriangleMesh`: `its` and `m_stats` (and `m_init_shift`).
///
/// FIDELITY-NOTE: the crate's `TriangleMeshStats` has no `non_manifold_edges`,
/// `non_manifold_vertices` or `has_reversed_faces` (TriangleMesh.hpp:57-61 at the pin), so they
/// are neither counted nor merged here.
#[derive(Clone, Debug)]
pub struct ModelMesh {
    /// TriangleMesh.hpp:166
    pub its: indexed_triangle_set,
    /// TriangleMesh.hpp:169
    m_stats: TriangleMeshStats,
    /// TriangleMesh.hpp:170
    m_init_shift: Vec3d,
}

impl Default for ModelMesh {
    fn default() -> Self {
        Self { its: indexed_triangle_set::default(), m_stats: TriangleMeshStats::default(), m_init_shift: Vec3d::zeros() }
    }
}

impl ModelMesh {
    /// TriangleMesh.hpp:97 `TriangleMesh() = default;`
    pub fn new() -> Self {
        Self::default()
    }

    /// TriangleMesh.cpp:89-92 `TriangleMesh(const indexed_triangle_set &its)`
    pub fn from_its(its: indexed_triangle_set) -> Self {
        let mut stats = TriangleMeshStats::default();
        crate::triangle_mesh::fill_initial_stats(&its, &mut stats);
        Self { its, m_stats: stats, m_init_shift: Vec3d::zeros() }
    }

    /// TriangleMesh.cpp:94-98 `TriangleMesh(indexed_triangle_set &&its, const RepairedMeshErrors& errors)`
    pub fn from_its_with_errors(its: indexed_triangle_set, errors: crate::triangle_mesh::RepairedMeshErrors) -> Self {
        let mut stats = TriangleMeshStats::default();
        stats.repaired_errors = errors;
        crate::triangle_mesh::fill_initial_stats(&its, &mut stats);
        Self { its, m_stats: stats, m_init_shift: Vec3d::zeros() }
    }

    /// The crate's f64 mesh as an `indexed_triangle_set` (the vertices back to `float`).
    pub fn from_triangle_mesh(mesh: &TriangleMesh) -> Self {
        let mut its = indexed_triangle_set::default();
        its.vertices = mesh.vertices().iter().map(|v| Vec3f::new(v.x as f32, v.y as f32, v.z as f32)).collect();
        its.indices = mesh
            .indices()
            .iter()
            .map(|t| Vec3i::new(t.indices[0] as i32, t.indices[1] as i32, t.indices[2] as i32))
            .collect();
        Self::from_its(its)
    }

    /// The crate's f64 mesh of these vertices and faces.
    pub fn to_triangle_mesh(&self) -> TriangleMesh {
        let vertices = self.its.vertices.iter().map(|v| Point3F::new(v.x as f64, v.y as f64, v.z as f64)).collect();
        let indices = self
            .its
            .indices
            .iter()
            .map(|f| crate::triangle_mesh::Triangle::new(f[0] as u32, f[1] as u32, f[2] as u32))
            .collect();
        TriangleMesh::from_parts(vertices, indices)
    }

    /// TriangleMesh.hpp:101
    pub fn clear(&mut self) {
        self.its = indexed_triangle_set::default();
        self.m_stats.clear();
    }

    /// TriangleMesh.cpp:255-260
    pub fn volume(&mut self) -> f32 {
        if self.m_stats.volume == -1.0 {
            self.m_stats.volume = crate::triangle_mesh::its_volume(&self.its);
        }
        self.m_stats.volume
    }

    /// TriangleMesh.cpp:272-294 `scale(const Vec3f &versor)`
    pub fn scale(&mut self, versor: &Vec3f) {
        // Scale extents.
        let s = *versor;
        self.m_stats.min = self.m_stats.min.component_mul(&s);
        self.m_stats.max = self.m_stats.max.component_mul(&s);
        // Scale size.
        self.m_stats.size = self.m_stats.size.component_mul(&s);
        // Scale volume.
        if self.m_stats.volume > 0.0 {
            self.m_stats.volume *= s[0] * s[1] * s[2];
        }
        if versor.x == versor.y && versor.x == versor.z {
            let s = versor.x;
            for v in &mut self.its.vertices {
                *v *= s;
            }
        } else {
            for v in &mut self.its.vertices {
                v.x *= versor.x;
                v.y *= versor.y;
                v.z *= versor.z;
            }
        }
    }

    /// TriangleMesh.cpp:296-304 `translate(const Vec3f &displacement)`
    pub fn translate(&mut self, displacement: &Vec3f) {
        if displacement.x != 0.0 || displacement.y != 0.0 || displacement.z != 0.0 {
            for v in &mut self.its.vertices {
                *v += displacement;
            }
            self.m_stats.min += displacement;
            self.m_stats.max += displacement;
        }
    }

    /// TriangleMesh.cpp:336-360 `mirror(const Axis axis)`
    pub fn mirror(&mut self, axis: Axis) {
        let iaxis = axis as usize;
        for v in &mut self.its.vertices {
            v[iaxis] *= -1.0;
        }
        crate::triangle_mesh::its_flip_triangles(&mut self.its);
        let tmp = self.m_stats.min[iaxis];
        self.m_stats.min[iaxis] = self.m_stats.max[iaxis];
        self.m_stats.max[iaxis] = tmp;
        self.m_stats.min[iaxis] *= -1.0;
        self.m_stats.max[iaxis] *= -1.0;
    }

    /// TriangleMesh.cpp:37-43 `update_bounding_box`
    fn update_bounding_box(&mut self) {
        crate::triangle_mesh::update_bounding_box(&self.its, &mut self.m_stats);
    }

    /// TriangleMesh.cpp:362-372 `transform(const Transform3d& t, bool fix_left_handed)`
    pub fn transform(&mut self, t: &Transform3d, fix_left_handed: bool) {
        // admesh/stl.h:372-380 its_transform(its, t)
        for v in &mut self.its.vertices {
            let p = eigen_transform_point(t, &Vec3d::new(v.x as f64, v.y as f64, v.z as f64));
            *v = Vec3f::new(p.x as f32, p.y as f32, p.z as f32);
        }
        let mut det = eigen_determinant3(&linear_of(t));
        if fix_left_handed && det < 0.0 {
            crate::triangle_mesh::its_flip_triangles(&mut self.its);
            det = -det;
        }
        self.m_stats.volume = (self.m_stats.volume as f64 * det) as f32;
        self.update_bounding_box();
    }

    /// TriangleMesh.cpp:374-384 `transform(const Matrix3d& m, bool fix_left_handed)`
    pub fn transform_matrix3(&mut self, m: &Matrix3d, fix_left_handed: bool) {
        // admesh/stl.h:383-390 its_transform(its, m)
        for v in &mut self.its.vertices {
            let p = eigen_matrix3_mul_vec(m, &Vec3d::new(v.x as f64, v.y as f64, v.z as f64));
            *v = Vec3f::new(p.x as f32, p.y as f32, p.z as f32);
        }
        let mut det = eigen_determinant3(m);
        if fix_left_handed && det < 0.0 {
            crate::triangle_mesh::its_flip_triangles(&mut self.its);
            det = -det;
        }
        self.m_stats.volume = (self.m_stats.volume as f64 * det) as f32;
        self.update_bounding_box();
    }

    /// TriangleMesh.cpp:386-399 `flip_triangles()`
    ///
    /// FIDELITY-NOTE: `m_stats.has_reversed_faces` (its_detect_reversed_faces) is not kept.
    pub fn flip_triangles(&mut self) {
        crate::triangle_mesh::its_flip_triangles(&mut self.its);
        self.m_stats.volume = -self.m_stats.volume;
    }

    /// TriangleMesh.cpp:419-422
    pub fn is_splittable(&self) -> bool {
        crate::triangle_mesh::its_is_splittable(&self.its)
    }

    /// TriangleMesh.cpp:424-441 `split(float scale_det)`, `MIN_MESH_VOLUME = 1e-2f`
    pub fn split(&self, scale_det: f32) -> Vec<ModelMesh> {
        const MIN_MESH_VOLUME: f32 = 1e-2;
        let itss = crate::triangle_mesh::its_split(&self.its);
        let mut out = Vec::with_capacity(itss.len());
        for m in itss {
            // The TriangleMesh constructor shall fill in the mesh statistics including volume.
            let mut temp_triangle_mesh = ModelMesh::from_its(m);
            if (temp_triangle_mesh.volume() * scale_det).abs() < MIN_MESH_VOLUME {
                continue;
            }
            if temp_triangle_mesh.volume() < 0.0 {
                // Some source mesh parts may be incorrectly oriented. Correct them.
                temp_triangle_mesh.flip_triangles();
            }
            out.push(temp_triangle_mesh);
        }
        out
    }

    /// TriangleMesh.cpp:443-465 `split_and_save_relationship(result, scale_det)`
    pub fn split_and_save_relationship(&self, result: &mut Vec<HashMap<i32, i32>>, scale_det: f32) -> Vec<ModelMesh> {
        const MIN_MESH_VOLUME: f32 = 1e-2;
        let neighbors = crate::triangle_mesh::its_face_neighbors(&self.its);
        let itss_and_ships = crate::mesh_split_impl::its_split_and_save_relationship(&self.its, &neighbors);
        let mut out = Vec::with_capacity(itss_and_ships.itses.len());
        result.reserve(itss_and_ships.itses.len());
        for (index, m) in itss_and_ships.itses.into_iter().enumerate() {
            // The TriangleMesh constructor shall fill in the mesh statistics including volume.
            let mut temp_triangle_mesh = ModelMesh::from_its(m);
            if (temp_triangle_mesh.volume() * scale_det).abs() < MIN_MESH_VOLUME {
                continue;
            }
            if temp_triangle_mesh.volume() < 0.0 {
                // Some source mesh parts may be incorrectly oriented. Correct them.
                temp_triangle_mesh.flip_triangles();
            }
            out.push(temp_triangle_mesh);
            result.push(itss_and_ships.ships[index].clone());
        }
        out
    }

    /// TriangleMesh.cpp:467-471 `merge(const TriangleMesh &mesh)`
    pub fn merge(&mut self, mesh: &ModelMesh) {
        crate::triangle_mesh::its_merge(&mut self.its, &mesh.its);
        self.m_stats = self.m_stats.merge(&mesh.m_stats);
    }

    /// TriangleMesh.cpp:492-499 `bounding_box()`: the stats extents as doubles.
    pub fn bounding_box(&self) -> BoundingBoxf3 {
        let mut bb = BoundingBoxf3::new();
        bb.defined = true;
        bb.min = BbVec3::new(self.m_stats.min.x as f64, self.m_stats.min.y as f64, self.m_stats.min.z as f64);
        bb.max = BbVec3::new(self.m_stats.max.x as f64, self.m_stats.max.y as f64, self.m_stats.max.z as f64);
        bb
    }

    /// TriangleMesh.cpp:501-507 `transformed_bounding_box(const Transform3d &trafo)`
    pub fn transformed_bounding_box(&self, trafo: &Transform3d) -> BoundingBoxf3 {
        let mut bbox = BoundingBoxf3::new();
        for v in &self.its.vertices {
            bbox_merge_point(&mut bbox, &eigen_transform_point(trafo, &Vec3d::new(v.x as f64, v.y as f64, v.z as f64)));
        }
        bbox
    }

    /// TriangleMesh.hpp:143 `Vec3d size() const`
    pub fn size(&self) -> Vec3d {
        Vec3d::new(self.m_stats.size.x as f64, self.m_stats.size.y as f64, self.m_stats.size.z as f64)
    }

    /// TriangleMesh.hpp:145 `Vec3d center() const`
    pub fn center(&self) -> Vec3d {
        from_bb(&self.bounding_box().center())
    }

    /// TriangleMesh.cpp:560-566 `convex_hull_3d()`: `TriangleMesh(its_convex_hull(this->its))`.
    ///
    /// FIDELITY-NOTE: `its_convex_hull` runs qhull ("Qt"), which the crate has no port of. This
    /// returns the hull of the same extreme vertices (quickhull, [`its_convex_hull`]); the vertex
    /// order and the triangulation of the hull faces are not qhull's.
    pub fn convex_hull_3d(&self) -> ModelMesh {
        ModelMesh::from_its(its_convex_hull(&self.its.vertices))
    }

    /// TriangleMesh.hpp:153
    pub fn facets_count(&self) -> usize {
        self.m_stats.number_of_facets as usize
    }

    /// TriangleMesh.hpp:154
    pub fn empty(&self) -> bool {
        self.facets_count() == 0
    }

    /// TriangleMesh.hpp:163
    pub fn stats(&self) -> &TriangleMeshStats {
        &self.m_stats
    }

    /// TriangleMesh.hpp:165
    pub fn set_init_shift(&mut self, offset: &Vec3d) {
        self.m_init_shift = *offset;
    }

    /// TriangleMesh.hpp:166
    pub fn get_init_shift(&self) -> Vec3d {
        self.m_init_shift
    }
}

/// The convex hull of a point set as a triangle set: the extreme points (those not inside the
/// hull nor on one of its faces or edges) and a triangulation of the hull surface. Stands in for
/// TriangleMesh.cpp:1737 `its_convex_hull` (qhull "Qt", see [`ModelMesh::convex_hull_3d`]).
///
/// Quickhull over `f64` copies of the `float` points; the orientation test is exact for the
/// differences of `float` coordinates up to the rounding of the final products.
pub fn its_convex_hull(pts: &[Vec3f]) -> indexed_triangle_set {
    let mut out = indexed_triangle_set::default();
    // de-duplicate the points
    let mut points: Vec<[f64; 3]> = pts.iter().map(|p| [p.x as f64, p.y as f64, p.z as f64]).collect();
    points.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    points.dedup();
    let n = points.len();
    if n < 4 {
        return out;
    }
    let sub = |a: &[f64; 3], b: &[f64; 3]| [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
    let cross = |u: &[f64; 3], v: &[f64; 3]| [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
    let dot = |u: &[f64; 3], v: &[f64; 3]| u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
    // signed volume of (a, b, c, p): > 0 when p is above the plane of the ccw triangle (a, b, c)
    let orient = |pts: &Vec<[f64; 3]>, a: usize, b: usize, c: usize, p: usize| {
        let n = cross(&sub(&pts[b], &pts[a]), &sub(&pts[c], &pts[a]));
        dot(&n, &sub(&pts[p], &pts[a]))
    };
    // initial simplex: extreme points along x, the farthest from their line, the farthest from
    // their plane
    let mut i0 = 0;
    let mut i1 = 0;
    for i in 0..n {
        if points[i][0] < points[i0][0] {
            i0 = i;
        }
        if points[i][0] > points[i1][0] {
            i1 = i;
        }
    }
    if i0 == i1 {
        i1 = if i0 == 0 { 1 } else { 0 };
    }
    let d01 = sub(&points[i1], &points[i0]);
    let mut i2 = usize::MAX;
    let mut best = 0.0;
    for i in 0..n {
        let c = cross(&d01, &sub(&points[i], &points[i0]));
        let d = dot(&c, &c);
        if d > best {
            best = d;
            i2 = i;
        }
    }
    if i2 == usize::MAX {
        return out;
    }
    let mut i3 = usize::MAX;
    let mut best = 0.0;
    for i in 0..n {
        let d = orient(&points, i0, i1, i2, i).abs();
        if d > best {
            best = d;
            i3 = i;
        }
    }
    if i3 == usize::MAX {
        return out;
    }
    // faces as ccw triples seen from outside
    let mut faces: Vec<[usize; 3]> = Vec::new();
    if orient(&points, i0, i1, i2, i3) > 0.0 {
        faces.extend_from_slice(&[[i0, i2, i1], [i0, i1, i3], [i1, i2, i3], [i2, i0, i3]]);
    } else {
        faces.extend_from_slice(&[[i0, i1, i2], [i0, i3, i1], [i1, i3, i2], [i2, i3, i0]]);
    }
    let mut alive = vec![true; faces.len()];
    // incremental insertion of the remaining points
    for p in 0..n {
        if p == i0 || p == i1 || p == i2 || p == i3 {
            continue;
        }
        let visible: Vec<usize> =
            (0..faces.len()).filter(|&f| alive[f] && orient(&points, faces[f][0], faces[f][1], faces[f][2], p) > 0.0).collect();
        if visible.is_empty() {
            continue;
        }
        // horizon edges: edges of visible faces whose twin is on a face that is not visible
        let mut edge_count: HashMap<(usize, usize), usize> = HashMap::new();
        for &f in &visible {
            let t = faces[f];
            for k in 0..3 {
                *edge_count.entry((t[k], t[(k + 1) % 3])).or_insert(0) += 1;
            }
        }
        let mut horizon: Vec<(usize, usize)> = Vec::new();
        for &f in &visible {
            let t = faces[f];
            for k in 0..3 {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                if !edge_count.contains_key(&(b, a)) {
                    horizon.push((a, b));
                }
            }
        }
        for &f in &visible {
            alive[f] = false;
        }
        for (a, b) in horizon {
            faces.push([a, b, p]);
            alive.push(true);
        }
    }
    // collect the faces and the vertices they use
    let mut map = vec![-1i32; n];
    for (f, t) in faces.iter().enumerate() {
        if !alive[f] {
            continue;
        }
        let mut idx = Vec3i::zeros();
        for k in 0..3 {
            if map[t[k]] == -1 {
                map[t[k]] = out.vertices.len() as i32;
                let p = points[t[k]];
                out.vertices.push(Vec3f::new(p[0] as f32, p[1] as f32, p[2] as f32));
            }
            idx[k] = map[t[k]];
        }
        out.indices.push(idx);
    }
    out
}

// =============================================================================================
// Model.hpp:781-848 — FacetsAnnotation
// =============================================================================================

/// Per-facet annotation storage (painted MMU segmentation, supports, seams).
///
/// C++: `class FacetsAnnotation : public ObjectWithTimestamp` (Model.hpp:781).
/// Wraps the same serialized form as `TriangleSelector::serialize()`:
/// `.0` = (triangle index, first bit offset), `.1` = bit stream
/// (TriangleSelector.hpp:275). The 3MF `paint_color` /
/// `slic3rpe:mmu_segmentation` triangle attribute is the hex encoding of one
/// triangle's slice of this stream.
///
/// This type is the `m_data` part, as other units build it from its data alone; the
/// `ObjectWithTimestamp` base a `ModelVolume` keeps for each of its annotations is
/// [`ModelFacetsAnnotation`].
#[derive(Debug, Clone, Default, PartialEq)]
pub struct FacetsAnnotation {
    /// `m_data` — Model.hpp:841.
    pub data: crate::triangle_selector::SerializedData,
}

impl FacetsAnnotation {
    /// Model.hpp:800 `bool empty() const { return m_data.first.empty(); }`
    pub fn is_empty(&self) -> bool {
        self.data.0.is_empty()
    }

    /// Number of triangles carrying an annotation.
    pub fn facet_count(&self) -> usize {
        self.data.0.len()
    }

    /// Model.hpp:786 `get_data()`
    pub fn get_data(&self) -> &crate::triangle_selector::SerializedData {
        &self.data
    }

    /// Model.cpp:4657-4666 `bool FacetsAnnotation::set(const TriangleSelector& selector)`: the
    /// selector's serialization replaces the data when it differs. `touch()` is
    /// [`ModelFacetsAnnotation::set`]'s.
    pub fn set(&mut self, selector: &crate::triangle_selector::TriangleSelector) -> bool {
        let sel_map = selector.serialize();
        if sel_map != self.data {
            self.data = sel_map;
            return true;
        }
        false
    }

    /// Model.cpp:4668-4676 `void FacetsAnnotation::reset()`
    pub fn reset(&mut self) {
        self.data.0.clear();
        self.data.1.clear();
    }

    /// Encode one triangle's state as a string of hexadecimal digits.
    /// Used for 3MF export; changing it breaks backwards compatibility.
    ///
    /// Model.cpp:4678-4702 `std::string FacetsAnnotation::get_triangle_as_string(int) const`
    pub fn get_triangle_as_string(&self, triangle_idx: i32) -> String {
        let mut out = String::new();

        let pos = self.data.0.partition_point(|&(tri, _)| tri < triangle_idx);
        if pos < self.data.0.len() && self.data.0[pos].0 == triangle_idx {
            let mut offset = self.data.0[pos].1 as usize;
            let end = if pos + 1 == self.data.0.len() {
                self.data.1.len()
            } else {
                self.data.0[pos + 1].1 as usize
            };
            while offset < end {
                let mut next_code = 0u32;
                for i in (0..4).rev() {
                    next_code <<= 1;
                    next_code |= u32::from(self.data.1[offset + i]);
                }
                offset += 4;
                debug_assert!(next_code <= 15);
                let digit = char::from_digit(next_code, 16).unwrap().to_ascii_uppercase();
                out.insert(0, digit);
            }
        }
        out
    }

    /// Recover triangle splitting & state from a string of hexadecimal values
    /// previously generated by [`Self::get_triangle_as_string`]. Used to load
    /// from 3MF (`paint_color` / `slic3rpe:mmu_segmentation` attributes).
    ///
    /// Triangle ids must be appended in ascending order (matches the C++
    /// assert `m_data.first.back().first < triangle_id`).
    ///
    /// Model.cpp:4704-4734 `void FacetsAnnotation::set_triangle_from_string(int, const std::string&)`
    pub fn set_triangle_from_string(&mut self, triangle_id: i32, s: &str) {
        debug_assert!(!s.is_empty());
        debug_assert!(self.data.0.last().map_or(true, |&(t, _)| t < triangle_id));
        self.data.0.push((triangle_id, self.data.1.len() as i32));

        for ch in s.chars().rev() {
            let dec = match ch {
                '0'..='9' => ch as u32 - '0' as u32,
                'A'..='F' => 10 + ch as u32 - 'A' as u32,
                _ => {
                    debug_assert!(false, "invalid facet-annotation hex digit {ch:?}");
                    continue;
                }
            };
            for i in 0..4 {
                self.data.1.push(dec & (1 << i) != 0);
            }
        }
    }

    /// Model.hpp:810 `void reserve(int n_triangles)`
    pub fn reserve(&mut self, n_triangles: i32) {
        self.data.0.reserve(n_triangles.max(0) as usize);
    }

    /// Model.hpp:814 `void shrink_to_fit()`
    pub fn shrink_to_fit(&mut self) {
        self.data.0.shrink_to_fit();
        self.data.1.shrink_to_fit();
    }

    /// Model.cpp:4736-4740 `bool FacetsAnnotation::equals(const FacetsAnnotation &other) const`
    pub fn equals(&self, other: &FacetsAnnotation) -> bool {
        let data = other.get_data();
        self.data == *data
    }
}

/// A `FacetsAnnotation` member of a `ModelVolume` with its `ObjectWithTimestamp` base
/// (Model.hpp:781): the ID and the timestamp the undo stack, `Print::apply()` and
/// `model_custom_*_data_changed()` compare.
#[derive(Debug, Clone)]
pub struct ModelFacetsAnnotation {
    base: ObjectWithTimestamp,
    facets: FacetsAnnotation,
}

impl std::ops::Deref for ModelFacetsAnnotation {
    type Target = FacetsAnnotation;
    fn deref(&self) -> &FacetsAnnotation {
        &self.facets
    }
}

impl std::ops::DerefMut for ModelFacetsAnnotation {
    fn deref_mut(&mut self) -> &mut FacetsAnnotation {
        &mut self.facets
    }
}

impl ModelFacetsAnnotation {
    /// Model.hpp:820 — default constructor assigns a unique ID.
    pub fn new() -> Self {
        Self { base: ObjectWithTimestamp::new(), facets: FacetsAnnotation::default() }
    }
    /// Model.hpp:823
    pub fn new_invalid() -> Self {
        Self { base: ObjectWithTimestamp::new_invalid(), facets: FacetsAnnotation::default() }
    }
    pub fn id(&self) -> ObjectID {
        self.base.id()
    }
    pub fn set_new_unique_id(&mut self) {
        self.base.set_new_unique_id();
    }
    pub fn timestamp(&self) -> Timestamp {
        self.base.timestamp()
    }
    pub fn timestamp_matches(&self, rhs: &ModelFacetsAnnotation) -> bool {
        self.base.timestamp_matches(&rhs.base)
    }
    /// The data part.
    pub fn annotation(&self) -> &FacetsAnnotation {
        &self.facets
    }
    /// Model.hpp:784 `assign(const FacetsAnnotation& rhs)`: the content when the timestamp differs.
    pub fn assign(&mut self, rhs: &ModelFacetsAnnotation) {
        if !self.base.timestamp_matches(&rhs.base) {
            self.facets.data = rhs.facets.data.clone();
            self.base.copy_timestamp(&rhs.base);
        }
    }
    /// Model.cpp:4657-4666
    pub fn set(&mut self, selector: &crate::triangle_selector::TriangleSelector) -> bool {
        if self.facets.set(selector) {
            self.base.touch();
            return true;
        }
        false
    }
    /// Model.cpp:4668-4676: clears the data and increases the timestamp.
    pub fn reset(&mut self) {
        self.facets.reset();
        self.base.touch();
    }
    /// ObjectID.hpp `ObjectWithTimestamp::touch()`: a new timestamp.
    pub fn touch(&mut self) {
        self.base.touch();
    }
    /// Replace the data and touch (the commit of a computed annotation).
    pub fn set_data(&mut self, data: FacetsAnnotation) {
        self.facets = data;
        self.base.touch();
    }

    /// `TriangleSelector selector(mv.mesh()); selector.deserialize(m_data, false);`
    fn selector_of(&self, mesh: &ModelMesh) -> crate::triangle_selector::TriangleSelector {
        let mut selector = crate::triangle_selector::TriangleSelector::new(mesh.to_triangle_mesh(), 0.6);
        // Reset of TriangleSelector is done inside TriangleSelector's constructor, so we don't need it to perform it again in deserialize().
        selector.deserialize(&self.facets.data, false, EnforcerBlockerType::EXTRUDER_MAX, EnforcerBlockerType::NONE, EnforcerBlockerType::NONE);
        selector
    }

    /// Model.cpp:4599-4605 `get_facets(const ModelVolume& mv, EnforcerBlockerType type)`: `mesh` is
    /// `mv.mesh()`.
    pub fn get_facets(&self, mesh: &ModelMesh, type_: EnforcerBlockerType) -> indexed_triangle_set {
        self.selector_of(mesh).get_facets(type_)
    }

    /// Model.cpp:4608-4613 `get_facets(const ModelVolume& mv, std::vector<indexed_triangle_set>& facets_per_type)`
    pub fn get_facets_per_type(&self, mesh: &ModelMesh, facets_per_type: &mut Vec<indexed_triangle_set>) {
        self.selector_of(mesh).get_facets_per_type(facets_per_type);
    }

    /// Model.cpp:4615-4622 `shift_states_above(const ModelVolume &mv, EnforcerBlockerType threshold, int delta)`
    pub fn shift_states_above(&mut self, mesh: &ModelMesh, threshold: EnforcerBlockerType, delta: i32) {
        if self.is_empty() {
            return;
        }
        let mut selector = self.selector_of(mesh);
        selector.shift_states_above(threshold, delta);
        self.set(&selector);
    }

    /// Model.cpp:4624-4632 `replace_state(const ModelVolume &mv, EnforcerBlockerType from, EnforcerBlockerType to)`:
    /// true when any facet changed.
    pub fn replace_state(&mut self, mesh: &ModelMesh, from: EnforcerBlockerType, to: EnforcerBlockerType) -> bool {
        if self.is_empty() || from == to || !self.has_facets(from) {
            return false;
        }
        let mut selector = self.selector_of(mesh);
        selector.replace_state(from, to);
        self.set(&selector)
    }

    /// Model.cpp:4634-4642 `set_enforcer_block_type_limit(mv, max_type, to_delete_filament, replace_filament)`
    pub fn set_enforcer_block_type_limit(
        &mut self,
        mesh: &ModelMesh,
        max_type: EnforcerBlockerType,
        to_delete_filament: EnforcerBlockerType,
        replace_filament: EnforcerBlockerType,
    ) {
        let mut selector = crate::triangle_selector::TriangleSelector::new(mesh.to_triangle_mesh(), 0.6);
        selector.deserialize(&self.facets.data, false, max_type, to_delete_filament, replace_filament);
        self.set(&selector);
    }

    /// Model.cpp:4644-4650 `get_facets_strict(const ModelVolume& mv, EnforcerBlockerType type)`
    pub fn get_facets_strict(&self, mesh: &ModelMesh, type_: EnforcerBlockerType) -> indexed_triangle_set {
        self.selector_of(mesh).get_facets_strict(type_)
    }

    /// Model.cpp:4652-4655 `has_facets(const ModelVolume& mv, EnforcerBlockerType type)`
    pub fn has_facets(&self, type_: EnforcerBlockerType) -> bool {
        crate::triangle_selector::TriangleSelector::has_facets_data(&self.facets.data, type_)
    }
}

// =============================================================================================
// Model.hpp:860-899 — RaycastResult, TextInfo; Model.hpp:908-919 — GlobalFeatureFilaments
// =============================================================================================

/// Model.hpp:870-899 `struct TextInfo`, the members Model.cpp reads.
///
/// FIDELITY-NOTE: `m_rr` (RaycastResult, the GUI's raycast of the text placement) is not carried.
#[derive(Debug, Clone, PartialEq)]
pub struct TextInfo {
    pub m_font_name: String,
    pub m_font_version: String,
    pub m_font_size: f32,
    pub m_curr_font_idx: i32,
    pub m_bold: bool,
    pub m_italic: bool,
    pub m_thickness: f32,
    pub m_embeded_depth: f32,
    pub m_rotate_angle: f32,
    pub m_text_gap: f32,
    pub m_is_surface_text: bool,
    pub m_keep_horizontal: bool,
    pub m_surface_type: i32,
    pub m_text: String,
    /// Model.hpp:898
    pub text_configuration: crate::text_configuration::TextConfiguration,
}

impl Default for TextInfo {
    fn default() -> Self {
        Self {
            m_font_name: String::new(),
            m_font_version: String::new(),
            m_font_size: 10.0,
            m_curr_font_idx: 0,
            m_bold: true,
            m_italic: false,
            m_thickness: 2.0,
            m_embeded_depth: 0.0,
            m_rotate_angle: 0.0,
            m_text_gap: 0.0,
            m_is_surface_text: false,
            m_keep_horizontal: false,
            m_surface_type: 1,
            m_text: String::new(),
            text_configuration: crate::text_configuration::TextConfiguration::default(),
        }
    }
}

/// Model.hpp:909-919 `struct GlobalFeatureFilaments`: 1-based feature filaments of the global
/// print config, 0 = Default.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct GlobalFeatureFilaments {
    pub wall_filament: i32,
    pub sparse_infill_filament: i32,
    pub solid_infill_filament: i32,
}

impl GlobalFeatureFilaments {
    /// Model.cpp:3399-3407: ids above filament_count are treated as Default.
    pub fn from_config(config: &dyn crate::config::ConfigBase, filament_count: usize) -> GlobalFeatureFilaments {
        let filament = |key: &str| {
            let opt = config.option(key);
            let id = opt.map(opt_get_int).unwrap_or(0);
            if id > 0 && id <= filament_count as i32 {
                id
            } else {
                0
            }
        };
        GlobalFeatureFilaments {
            wall_filament: filament("wall_filament"),
            sparse_infill_filament: filament("sparse_infill_filament"),
            solid_infill_filament: filament("solid_infill_filament"),
        }
    }

    /// Model.cpp:3409-3413: the filament count from filament_diameter of a full config.
    pub fn from_full_config(full_config: &dyn crate::config::ConfigBase) -> GlobalFeatureFilaments {
        let size = full_config.option("filament_diameter").and_then(|o| o.as_vector_base()).map(|v| v.size()).unwrap_or(0);
        Self::from_config(full_config, size)
    }
}

// =============================================================================================
// Model.hpp:923-1432 — ModelVolume
// =============================================================================================

/// Model.hpp:929-945 `struct ModelVolume::Source`: used by reload from disk.
#[derive(Debug, Clone, PartialEq)]
pub struct ModelVolumeSource {
    pub input_file: String,
    pub object_idx: i32,
    pub volume_idx: i32,
    pub mesh_offset: Vec3d,
    pub transform: Transformation,
    pub is_converted_from_inches: bool,
    pub is_converted_from_meters: bool,
    pub is_from_builtin_objects: bool,
}

impl Default for ModelVolumeSource {
    fn default() -> Self {
        Self {
            input_file: String::new(),
            object_idx: -1,
            volume_idx: -1,
            mesh_offset: Vec3d::zeros(),
            transform: Transformation::new(),
            is_converted_from_inches: false,
            is_converted_from_meters: false,
            is_from_builtin_objects: false,
        }
    }
}

/// Model.hpp:950-978 `struct ModelVolume::CutInfo`: information about connectors.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ModelVolumeCutInfo {
    pub is_from_upper: bool,
    pub is_connector: bool,
    pub is_processed: bool,
    pub connector_type: CutConnectorType,
    pub radius: f32,
    pub height: f32,
    /// [0.f : 1.f]
    pub radius_tolerance: f32,
    /// [0.f : 1.f]
    pub height_tolerance: f32,
    /// Index of the volume this piece was cut from (transient).
    pub source_part_idx: i32,
}

impl Default for ModelVolumeCutInfo {
    fn default() -> Self {
        Self {
            is_from_upper: true,
            is_connector: false,
            is_processed: true,
            connector_type: CutConnectorType::Plug,
            radius: 0.0,
            height: 0.0,
            radius_tolerance: 0.0,
            height_tolerance: 0.0,
            source_part_idx: -1,
        }
    }
}

impl ModelVolumeCutInfo {
    /// Model.hpp:969-972
    pub fn new(type_: CutConnectorType, radius: f32, height: f32, rad_tolerance: f32, h_tolerance: f32, processed: bool) -> Self {
        Self {
            is_connector: true,
            is_processed: processed,
            connector_type: type_,
            radius,
            height,
            radius_tolerance: rad_tolerance,
            height_tolerance: h_tolerance,
            ..Self::default()
        }
    }
    /// Model.hpp:974
    pub fn set_processed(&mut self) {
        self.is_processed = true;
    }
    /// Model.hpp:975
    pub fn invalidate(&mut self) {
        self.is_connector = false;
    }
    /// Model.hpp:976
    pub fn reset_from_upper(&mut self) {
        self.is_from_upper = true;
    }
}

/// Model.hpp:923 `class ModelVolume final : public ObjectBase`: an object STL, or a modifier
/// volume, over which a different set of parameters shall be applied.
#[derive(Debug, Clone)]
pub struct ModelVolume {
    base: ObjectBase,
    /// Model.hpp:926
    pub name: String,
    /// Model.hpp:946
    pub source: ModelVolumeSource,
    /// Model.hpp:979
    pub cut_info: ModelVolumeCutInfo,
    /// Model.hpp:1000
    pub config: ModelConfigObject,
    /// Model.hpp:1003 — list of mesh facets to be supported/unsupported.
    pub supported_facets: ModelFacetsAnnotation,
    /// Model.hpp:1006 — list of seam enforcers/blockers.
    pub seam_facets: ModelFacetsAnnotation,
    /// Model.hpp:1009 — list of mesh facets painted for MMU segmentation.
    pub mmu_segmentation_facets: ModelFacetsAnnotation,
    /// Model.hpp:1012 — list of fuzzy skin
    pub fuzzy_skin_facets: ModelFacetsAnnotation,
    /// Model.hpp:1014 — quick access for volume extruders, 1 based
    mmuseg_extruders: std::cell::RefCell<Vec<i32>>,
    /// Model.hpp:1015
    mmuseg_ts: Cell<Timestamp>,
    /// Model.hpp:1018 — list of exterior faces
    pub exterior_facets: ModelFacetsAnnotation,
    /// Model.hpp:1023 `std::optional<EmbossShape> emboss_shape`: set only when the volume is an
    /// embossed shape. FIDELITY-NOTE: only whether it is set is kept (`is_svg()`).
    pub emboss_shape: Option<()>,
    /// Model.hpp:1217
    m_mesh: Option<Arc<ModelMesh>>,
    /// Model.hpp:1219
    m_type: ModelVolumeType,
    /// Model.hpp:1220
    m_material_id: TModelMaterialId,
    /// Model.hpp:1221
    m_mmuseg_extruders_has_0_extruder: Cell<bool>,
    /// Model.hpp:1223
    m_convex_hull: Option<Arc<ModelMesh>>,
    /// Model.hpp:1225
    m_convex_hull_2d: std::cell::RefCell<Polygon>,
    /// Model.hpp:1226
    m_cached_trans_matrix: Cell<Transform3d>,
    /// Model.hpp:1227
    m_cached_2d_polygon: std::cell::RefCell<Polygon>,
    /// Model.hpp:1228
    m_transformation: Transformation,
    /// Model.hpp:1230
    m_assemble_transformation: Transformation,
    /// Model.hpp:1231
    m_assemble_initialized: bool,
    /// Model.hpp:1235
    m_part_guid: std::cell::RefCell<String>,
    /// Model.hpp:1237
    m_assembly_src_guid: String,
    /// Model.hpp:1240
    m_printable: bool,
    /// Model.hpp:1242
    m_text_info: TextInfo,
    /// Model.hpp:1244
    m_origin_mesh_or_vertice_render: bool,
    /// Model.hpp:1255 — -1 unknown, 0 not splittable, 1 splittable
    m_is_splittable: Cell<i32>,
    /// Model.hpp:1218 `ModelObject* object`: the parent object, held as its ID (FIDELITY-NOTE
    /// (back pointers) at the top of this file).
    object: Option<ObjectID>,
}

impl ModelVolume {
    /// The members every constructor starts from, the IDs allocated in the C++ declaration order:
    /// the volume, `config`, `supported_facets`, `seam_facets`, `mmu_segmentation_facets`,
    /// `fuzzy_skin_facets`, `exterior_facets`.
    fn with_fresh_ids(mesh: Option<Arc<ModelMesh>>, type_: ModelVolumeType) -> Self {
        let base = ObjectBase::new();
        let config = ModelConfigObject::new();
        let supported_facets = ModelFacetsAnnotation::new();
        let seam_facets = ModelFacetsAnnotation::new();
        let mmu_segmentation_facets = ModelFacetsAnnotation::new();
        let fuzzy_skin_facets = ModelFacetsAnnotation::new();
        let exterior_facets = ModelFacetsAnnotation::new();
        Self {
            base,
            name: String::new(),
            source: ModelVolumeSource::default(),
            cut_info: ModelVolumeCutInfo::default(),
            config,
            supported_facets,
            seam_facets,
            mmu_segmentation_facets,
            fuzzy_skin_facets,
            mmuseg_extruders: std::cell::RefCell::new(Vec::new()),
            mmuseg_ts: Cell::new(0),
            exterior_facets,
            emboss_shape: None,
            m_mesh: mesh,
            m_type: type_,
            m_material_id: String::new(),
            m_mmuseg_extruders_has_0_extruder: Cell::new(true),
            m_convex_hull: None,
            m_convex_hull_2d: std::cell::RefCell::new(Polygon::new()),
            m_cached_trans_matrix: Cell::new(Transform3d::identity()),
            m_cached_2d_polygon: std::cell::RefCell::new(Polygon::new()),
            m_transformation: Transformation::new(),
            m_assemble_transformation: Transformation::new(),
            m_assemble_initialized: false,
            m_part_guid: std::cell::RefCell::new(String::new()),
            m_assembly_src_guid: String::new(),
            m_printable: true,
            m_text_info: TextInfo::default(),
            m_origin_mesh_or_vertice_render: true,
            m_is_splittable: Cell::new(-1),
            object: None,
        }
    }

    /// Model.hpp:1257-1272 `ModelVolume(ModelObject *object, const TriangleMesh &mesh, ModelVolumeType type)`
    pub fn new(mesh: ModelMesh, type_: ModelVolumeType) -> Self {
        let facets = mesh.facets_count();
        let mut v = Self::with_fresh_ids(Some(Arc::new(mesh)), type_);
        if facets > 1 {
            v.calculate_convex_hull();
        }
        v
    }

    /// Model.hpp:1273-1286 `ModelVolume(ModelObject *object, const std::shared_ptr<const TriangleMesh> &mesh, ModelVolumeType type)`
    pub fn new_shared(mesh: Arc<ModelMesh>, type_: ModelVolumeType) -> Self {
        Self::with_fresh_ids(Some(mesh), type_)
    }

    /// Model.hpp:1287-1300 `ModelVolume(ModelObject *object, TriangleMesh &&mesh, TriangleMesh &&convex_hull, ModelVolumeType type)`
    pub fn new_with_hull(mesh: ModelMesh, convex_hull: ModelMesh, type_: ModelVolumeType) -> Self {
        let mut v = Self::with_fresh_ids(Some(Arc::new(mesh)), type_);
        v.m_convex_hull = Some(Arc::new(convex_hull));
        v
    }

    /// Model.hpp:1303-1335 `ModelVolume(ModelObject *object, const ModelVolume &other)`: a copy
    /// with the IDs of `other`, except `exterior_facets`, which is default constructed (a new ID).
    ///
    /// FIDELITY-NOTE: `set_material_id(other.material_id())` registers the material in the
    /// parent's model; the caller with the model does that (`Model::add_material`).
    pub fn new_copy_of(other: &ModelVolume) -> Self {
        let exterior_facets = ModelFacetsAnnotation::new();
        Self {
            base: other.base,
            name: other.name.clone(),
            source: other.source.clone(),
            cut_info: ModelVolumeCutInfo::default(),
            config: other.config.clone(),
            supported_facets: other.supported_facets.clone(),
            seam_facets: other.seam_facets.clone(),
            mmu_segmentation_facets: other.mmu_segmentation_facets.clone(),
            fuzzy_skin_facets: other.fuzzy_skin_facets.clone(),
            mmuseg_extruders: std::cell::RefCell::new(Vec::new()),
            mmuseg_ts: Cell::new(0),
            exterior_facets,
            emboss_shape: other.emboss_shape,
            m_mesh: other.m_mesh.clone(),
            m_type: other.m_type,
            m_material_id: other.m_material_id.clone(),
            m_mmuseg_extruders_has_0_extruder: Cell::new(true),
            m_convex_hull: other.m_convex_hull.clone(),
            m_convex_hull_2d: std::cell::RefCell::new(Polygon::new()),
            m_cached_trans_matrix: Cell::new(Transform3d::identity()),
            m_cached_2d_polygon: std::cell::RefCell::new(Polygon::new()),
            m_transformation: other.m_transformation,
            m_assemble_transformation: other.m_assemble_transformation,
            m_assemble_initialized: other.m_assemble_initialized,
            m_part_guid: std::cell::RefCell::new(other.m_part_guid.borrow().clone()),
            m_assembly_src_guid: other.m_assembly_src_guid.clone(),
            m_printable: other.m_printable,
            m_text_info: other.m_text_info.clone(),
            m_origin_mesh_or_vertice_render: true,
            m_is_splittable: Cell::new(-1),
            object: None,
        }
    }

    /// Model.hpp:1337-1372 `ModelVolume(ModelObject *object, const ModelVolume &other, const TriangleMesh &&mesh)`:
    /// a new mesh, therefore a new ID; the facets new and empty, the config copied then given a
    /// new ID (IDs in the order: volume, the five facets, config).
    pub fn new_copy_with_mesh(other: &ModelVolume, mesh: ModelMesh) -> Self {
        let base = ObjectBase::new();
        let config = other.config.clone();
        let supported_facets = ModelFacetsAnnotation::new();
        let seam_facets = ModelFacetsAnnotation::new();
        let mmu_segmentation_facets = ModelFacetsAnnotation::new();
        let fuzzy_skin_facets = ModelFacetsAnnotation::new();
        let exterior_facets = ModelFacetsAnnotation::new();
        let facets = mesh.facets_count();
        let mut v = Self {
            base,
            name: other.name.clone(),
            source: other.source.clone(),
            cut_info: ModelVolumeCutInfo::default(),
            config,
            supported_facets,
            seam_facets,
            mmu_segmentation_facets,
            fuzzy_skin_facets,
            mmuseg_extruders: std::cell::RefCell::new(Vec::new()),
            mmuseg_ts: Cell::new(0),
            exterior_facets,
            emboss_shape: other.emboss_shape,
            m_mesh: Some(Arc::new(mesh)),
            m_type: other.m_type,
            m_material_id: other.m_material_id.clone(),
            m_mmuseg_extruders_has_0_extruder: Cell::new(true),
            m_convex_hull: None,
            m_convex_hull_2d: std::cell::RefCell::new(Polygon::new()),
            m_cached_trans_matrix: Cell::new(Transform3d::identity()),
            m_cached_2d_polygon: std::cell::RefCell::new(Polygon::new()),
            m_transformation: other.m_transformation,
            m_assemble_transformation: other.m_assemble_transformation,
            m_assemble_initialized: other.m_assemble_initialized,
            m_part_guid: std::cell::RefCell::new(String::new()),
            m_assembly_src_guid: String::new(),
            m_printable: other.m_printable,
            m_text_info: TextInfo::default(),
            m_origin_mesh_or_vertice_render: true,
            m_is_splittable: Cell::new(-1),
            object: None,
        };
        v.config.set_new_unique_id();
        if facets > 1 {
            v.calculate_convex_hull();
        }
        v
    }

    pub fn id(&self) -> ObjectID {
        self.base.id()
    }

    /// Model.hpp:1024 `ModelObject* get_object() const { return this->object; }`: the ID of the
    /// parent object, `None` for `nullptr`.
    pub fn get_object(&self) -> Option<ObjectID> {
        self.object
    }

    /// Model.hpp:1208 `void set_model_object(ModelObject *model_object) { object = model_object; }`:
    /// `model_object` is the ID of the parent object, `None` for `nullptr`.
    pub fn set_model_object(&mut self, model_object: Option<ObjectID>) {
        self.object = model_object;
    }

    /// Model.hpp:981
    pub fn is_from_upper(&self) -> bool {
        self.cut_info.is_from_upper
    }
    /// Model.hpp:982
    pub fn reset_from_upper(&mut self) {
        self.cut_info.reset_from_upper();
    }
    /// Model.hpp:984
    pub fn is_cut_connector(&self) -> bool {
        self.cut_info.is_processed && self.cut_info.is_connector
    }
    /// Model.hpp:985
    pub fn invalidate_cut_info(&mut self) {
        self.cut_info.invalidate();
    }

    /// Model.hpp:988 — the triangular model.
    pub fn mesh(&self) -> &ModelMesh {
        self.m_mesh.as_deref().expect("ModelVolume::mesh(): no mesh")
    }
    /// Model.hpp:989
    pub fn mesh_ptr(&self) -> Option<&ModelMesh> {
        self.m_mesh.as_deref()
    }
    /// Model.hpp:991 `set_mesh(TriangleMesh &&mesh)`
    pub fn set_mesh(&mut self, mesh: ModelMesh) {
        self.m_mesh = Some(Arc::new(mesh));
    }
    /// Model.hpp:994 `set_mesh(std::shared_ptr<const TriangleMesh> &mesh)`
    pub fn set_mesh_shared(&mut self, mesh: Arc<ModelMesh>) {
        self.m_mesh = Some(mesh);
    }
    /// Model.hpp:996
    pub fn reset_mesh(&mut self) {
        self.m_mesh = Some(Arc::new(ModelMesh::new()));
    }
    /// Model.hpp:997
    pub fn get_mesh_shared_ptr(&self) -> Option<&Arc<ModelMesh>> {
        self.m_mesh.as_ref()
    }

    /// Model.hpp:1026
    pub fn type_(&self) -> ModelVolumeType {
        self.m_type
    }
    /// Model.hpp:1027
    pub fn set_type(&mut self, t: ModelVolumeType) {
        self.m_type = t;
    }
    /// Model.hpp:1028
    pub fn is_model_part(&self) -> bool {
        self.m_type == ModelVolumeType::ModelPart
    }
    /// Model.hpp:1029
    pub fn is_negative_volume(&self) -> bool {
        self.m_type == ModelVolumeType::NegativeVolume
    }
    /// Model.hpp:1030
    pub fn is_modifier(&self) -> bool {
        self.m_type == ModelVolumeType::ParameterModifier
    }
    /// Model.hpp:1031
    pub fn is_support_enforcer(&self) -> bool {
        self.m_type == ModelVolumeType::SupportEnforcer
    }
    /// Model.hpp:1032
    pub fn is_support_blocker(&self) -> bool {
        self.m_type == ModelVolumeType::SupportBlocker
    }
    /// Model.hpp:1033
    pub fn is_support_modifier(&self) -> bool {
        self.m_type == ModelVolumeType::SupportBlocker || self.m_type == ModelVolumeType::SupportEnforcer
    }
    /// Model.hpp:1034
    pub fn is_svg(&self) -> bool {
        self.emboss_shape.is_some()
    }
    /// Model.hpp:1036
    pub fn material_id(&self) -> TModelMaterialId {
        self.m_material_id.clone()
    }

    /// Model.cpp:3945-3947
    pub fn set_text_configuration(&mut self, text_configuration: crate::text_configuration::TextConfiguration) {
        self.m_text_info.text_configuration = text_configuration;
    }
    /// Model.hpp:1160
    pub fn get_text_configuration(&mut self) -> &mut crate::text_configuration::TextConfiguration {
        &mut self.m_text_info.text_configuration
    }
    /// Model.cpp:3949-3965
    pub fn check_boldness_skew_min_max(&mut self, min_boldness: f32, max_boldness: f32, min_skew: f32, max_skew: f32) {
        let prop = &mut self.m_text_info.text_configuration.style.prop;
        let temp_custom_boldness = prop.boldness.unwrap_or(0.0);
        if temp_custom_boldness > max_boldness {
            prop.boldness = Some(0.0);
        } else if temp_custom_boldness < min_boldness {
            prop.boldness = Some(0.0);
        }

        let temp_custom_skew = prop.skew.unwrap_or(0.0);
        if temp_custom_skew > max_skew {
            prop.skew = Some(0.0);
        } else if temp_custom_skew < min_skew {
            prop.skew = Some(0.0);
        }
    }
    /// Model.hpp:1161
    pub fn set_text_info(&mut self, text_info: TextInfo) {
        self.m_text_info = text_info;
    }
    /// Model.hpp:1162
    pub fn clear_text_info(&mut self) {
        self.m_text_info.m_text = String::new();
    }
    /// Model.hpp:1163
    pub fn get_text_info(&self) -> &TextInfo {
        &self.m_text_info
    }
    /// Model.hpp:1165
    pub fn is_text(&self) -> bool {
        !self.m_text_info.m_text.is_empty()
    }

    /// Model.hpp:1167-1174 — ModelVolume, config and four facets (not exterior_facets).
    pub fn set_new_unique_id(&mut self) {
        self.base.set_new_unique_id();
        self.config.set_new_unique_id();
        self.supported_facets.set_new_unique_id();
        self.fuzzy_skin_facets.set_new_unique_id();
        self.seam_facets.set_new_unique_id();
        self.mmu_segmentation_facets.set_new_unique_id();
    }

    /// Model.hpp:1176
    pub fn is_fdm_support_painted(&self) -> bool {
        !self.supported_facets.is_empty()
    }
    /// Model.hpp:1177
    pub fn is_fuzzy_skin_facets_painted(&self) -> bool {
        !self.fuzzy_skin_facets.is_empty()
    }
    /// Model.hpp:1178
    pub fn is_seam_painted(&self) -> bool {
        !self.seam_facets.is_empty()
    }
    /// Model.hpp:1179
    pub fn is_mm_painted(&self) -> bool {
        !self.mmu_segmentation_facets.is_empty()
    }
    /// Model.hpp:1181-1183
    pub fn set_origin_mesh_render_type(&mut self, is_mesh: bool) {
        self.m_origin_mesh_or_vertice_render = is_mesh;
    }
    /// Model.hpp:1184
    pub fn get_origin_mesh_or_vertice_render(&self) -> bool {
        self.m_origin_mesh_or_vertice_render
    }
    /// Model.hpp:1187
    pub fn part_guid(&self) -> String {
        self.m_part_guid.borrow().clone()
    }
    /// Model.hpp:1188
    pub fn set_part_guid(&mut self, guid: &str) {
        *self.m_part_guid.borrow_mut() = guid.to_string();
    }
    /// Model.hpp:1192
    pub fn assembly_src_guid(&self) -> &str {
        &self.m_assembly_src_guid
    }
    /// Model.hpp:1193
    pub fn set_assembly_src_guid(&mut self, guid: &str) {
        self.m_assembly_src_guid = guid.to_string();
    }
    /// Model.hpp:1197
    pub fn printable(&self) -> bool {
        self.m_printable
    }
    /// Model.hpp:1198
    pub fn set_printable(&mut self, v: bool) {
        self.m_printable = v;
    }

    /// Model.cpp:2215-2221 `ensure_part_guid(bool force)`: lazily assign a stable identity.
    ///
    /// FIDELITY-NOTE: a random version-4 UUID text as `boost::uuids::random_generator`, from the
    /// standard library's per-process random hasher keys instead of the system entropy source.
    pub fn ensure_part_guid(&self, force: bool) -> String {
        if self.m_part_guid.borrow().is_empty() || force {
            *self.m_part_guid.borrow_mut() = random_uuid_string();
        }
        self.m_part_guid.borrow().clone()
    }

    /// Model.cpp:2223-2236 `is_the_only_one_part()`: behave like an object. `object` is the parent.
    pub fn is_the_only_one_part(&self, object: Option<&ModelObject>) -> bool {
        if self.m_type != ModelVolumeType::ModelPart {
            return false;
        }
        let object = match object {
            None => return false,
            Some(o) => o,
        };
        for v in &object.volumes {
            // is this volume?
            if v.id() == self.id() {
                continue;
            }
            // exist another model part in object?
            if v.type_() == ModelVolumeType::ModelPart {
                return false;
            }
        }
        true
    }

    /// Model.cpp:3252-3258 `set_material_id`: `materials` is the parent model's material map.
    pub fn set_material_id(&mut self, material_id: TModelMaterialId, materials: &mut ModelMaterialMap) {
        self.m_material_id = material_id.clone();
        // ensure m_material_id references an existing material
        if !material_id.is_empty() {
            materials_add(materials, &material_id);
        }
    }

    /// Model.cpp:3260-3265
    pub fn reset_extra_facets(&mut self) {
        self.supported_facets.reset();
        self.fuzzy_skin_facets.reset();
        self.seam_facets.reset();
        self.mmu_segmentation_facets.reset();
    }

    /// Model.cpp:3268-3298 `reproject_paint_keep(new_mesh, out, progress, cancel)`: re-project the
    /// four paint layers from the current mesh onto `new_mesh` into `out`, without changing this
    /// volume; false when canceled. `object` is the parent (its first instance scales the
    /// subdivision floor).
    pub fn reproject_paint_keep(
        &self,
        new_mesh: &ModelMesh,
        out: &mut crate::paint_reproject::PaintKeepPrepared,
        progress: Option<&mut crate::paint_reproject::PaintReprojectProgressCallback<'_>>,
        cancel: Option<&mut crate::paint_reproject::PaintReprojectCancelCallback<'_>>,
        object: Option<&ModelObject>,
    ) -> bool {
        let old_mesh = self.mesh();
        // instance 0 stands in for the selection the gizmo would have used, and only its scaling matters here.
        let instance = object.and_then(|o| o.instances.first());
        let dst_world_matrix = eigen_mul_affine(
            &instance.map(|i| i.get_transformation().get_matrix_no_offset()).unwrap_or_else(Transform3d::identity),
            &self.matrix(),
        );
        crate::paint_reproject::reproject_paint_geometric(
            &old_mesh.its,
            self.supported_facets.annotation(),
            self.seam_facets.annotation(),
            self.mmu_segmentation_facets.annotation(),
            self.fuzzy_skin_facets.annotation(),
            &new_mesh.its,
            &Transform3d::identity(),
            &mut out.supported,
            &mut out.seam,
            &mut out.mmu,
            &mut out.fuzzy,
            progress,
            cancel,
            Some(&dst_world_matrix),
        )
    }

    /// Model.cpp:3300-3310 `commit_mesh_keep_paint(PaintKeepPrepared &&prepared)`: the new mesh and
    /// its re-projected paint layers.
    ///
    /// FIDELITY-NOTE: `assign()` copies the prepared layer's timestamp; the prepared layers here
    /// are data only, so each layer is given a new timestamp instead.
    pub fn commit_mesh_keep_paint(&mut self, prepared: crate::paint_reproject::PaintKeepPrepared) {
        self.set_mesh(ModelMesh::from_its(prepared.mesh));
        self.supported_facets.set_data(prepared.supported);
        self.seam_facets.set_data(prepared.seam);
        self.mmu_segmentation_facets.set_data(prepared.mmu);
        self.fuzzy_skin_facets.set_data(prepared.fuzzy);
    }

    /// Model.cpp:3312-3322 `set_mesh_keep_paint(TriangleMesh &&mesh_in, progress, cancel)`: false
    /// when canceled (mesh and paint unchanged).
    pub fn set_mesh_keep_paint(
        &mut self,
        mesh_in: ModelMesh,
        progress: Option<&mut crate::paint_reproject::PaintReprojectProgressCallback<'_>>,
        cancel: Option<&mut crate::paint_reproject::PaintReprojectCancelCallback<'_>>,
        object: Option<&ModelObject>,
    ) -> bool {
        let mut prepared = crate::paint_reproject::PaintKeepPrepared::default();
        if !self.reproject_paint_keep(&mesh_in, &mut prepared, progress, cancel, object) {
            return false;
        }
        // prepared.mesh = std::move(mesh_in); commit_mesh_keep_paint(std::move(prepared)) — the
        // mesh committed as it came, with its stats
        self.set_mesh(mesh_in);
        self.supported_facets.set_data(prepared.supported);
        self.seam_facets.set_data(prepared.seam);
        self.mmu_segmentation_facets.set_data(prepared.mmu);
        self.fuzzy_skin_facets.set_data(prepared.fuzzy);
        true
    }

    /// Model.cpp:3325-3328 `material()`: the material of the parent model.
    pub fn material<'a>(&self, model: &'a Model) -> Option<&'a ModelMaterial> {
        model.get_material(&self.m_material_id)
    }

    /// Model.cpp:3330-3335 `set_material(material_id, const ModelMaterial &material)`
    pub fn set_material(&mut self, material_id: TModelMaterialId, material: &ModelMaterial, materials: &mut ModelMaterialMap) {
        self.m_material_id = material_id.clone();
        if !material_id.is_empty() {
            materials_add_copy(materials, &material_id, material);
        }
    }

    /// Model.cpp:3338-3349 `extruder_id()`: this volume's config, then the parent object's.
    pub fn extruder_id(&self, object: &ModelObject) -> i32 {
        let mut opt = self.config.option("extruder");
        if opt.is_none() || opt.map(opt_get_int) == Some(0) {
            opt = object.config.option("extruder");
        }
        match opt {
            None => 1,
            Some(o) => opt_get_int(o),
        }
    }

    /// Model.cpp:3351-3358 `is_splittable()`: cached, as `mesh.is_splittable()` is expensive.
    pub fn is_splittable(&self) -> bool {
        if self.m_is_splittable.get() == -1 {
            self.m_is_splittable.set(crate::triangle_mesh::its_is_splittable(&self.mesh().its) as i32);
        }
        self.m_is_splittable.get() == 1
    }

    /// Model.cpp:3360-3397 `apply_tolerance()`
    pub fn apply_tolerance(&mut self) {
        // assert(self.cut_info.is_connector): compiled out in the reference build (-DNDEBUG)
        if !self.cut_info.is_processed {
            return;
        }

        let mut sf = self.get_scaling_factor();
        // make a "hole" wider
        let size_scale = if (self.cut_info.radius - 0.0).abs() < EPSILON as f32 {
            // For compatibility with old files
            1.0 + self.cut_info.radius_tolerance as f64
        } else {
            (self.cut_info.radius as f64 + self.cut_info.radius_tolerance as f64) / self.cut_info.radius as f64
        };

        sf[0] *= size_scale;
        sf[1] *= size_scale;

        // make a "hole" dipper
        let height_scale = if (self.cut_info.height - 0.0).abs() < EPSILON as f32 {
            // For compatibility with old files
            1.0 + self.cut_info.height_tolerance as f64
        } else {
            (self.cut_info.height as f64 + self.cut_info.height_tolerance as f64) / self.cut_info.height as f64
        };

        sf[2] *= height_scale;

        self.set_scaling_factor(&sf);

        // correct offset in respect to the new depth
        let rot = crate::geometry::geometry::rotation_transform(&self.get_rotation());
        let mut rot_norm = eigen_transform_point(&rot, &Vec3d::new(0.0, 0.0, 1.0));
        let norm = eigen_squared_norm3(&rot_norm).sqrt();
        if norm != 0.0 {
            rot_norm /= norm;
        }

        let mut z_offset = 0.5 * self.cut_info.height_tolerance as f64;
        if self.cut_info.connector_type == CutConnectorType::Plug || self.cut_info.connector_type == CutConnectorType::Snap {
            z_offset -= 0.05; // add small Z offset to better preview
        }

        let offset = self.get_offset() + rot_norm * z_offset;
        self.set_offset(&offset);
    }

    /// Model.cpp:3416-3466 `get_extruders(global_features)`: `object` is the parent.
    pub fn get_extruders(&self, object: &ModelObject, global_features: &GlobalFeatureFilaments) -> Vec<i32> {
        if self.m_type == ModelVolumeType::Invalid
            || self.m_type == ModelVolumeType::NegativeVolume
            || self.m_type == ModelVolumeType::SupportBlocker
            || self.m_type == ModelVolumeType::SupportEnforcer
        {
            return Vec::new();
        }

        if self.mmu_segmentation_facets.timestamp() != self.mmuseg_ts.get() {
            let mut its_per_type: Vec<indexed_triangle_set> = Vec::new();
            let mut mmuseg_extruders = self.mmuseg_extruders.borrow_mut();
            mmuseg_extruders.clear();
            self.mmuseg_ts.set(self.mmu_segmentation_facets.timestamp());
            self.mmu_segmentation_facets.get_facets_per_type(self.mesh(), &mut its_per_type);
            for (idx, its) in its_per_type.iter().enumerate().skip(1) {
                if its.indices.is_empty() {
                    continue;
                }
                mmuseg_extruders.push(idx as i32);
            }
            if !its_per_type.is_empty() && its_per_type[0].indices.is_empty() {
                self.m_mmuseg_extruders_has_0_extruder.set(false);
            } else {
                self.m_mmuseg_extruders_has_0_extruder.set(true);
            }
        }

        let mut volume_extruders = self.mmuseg_extruders.borrow().clone();
        let volume_extruder_id = self.extruder_id(object);
        // Slicing ignores MMU paint on non-model-part volumes (modifiers, etc.),
        // so their volume extruder is always effective regardless of paint coverage.
        let paint_affects_slicing = self.is_model_part();
        if (!paint_affects_slicing || self.m_mmuseg_extruders_has_0_extruder.get()) && volume_extruder_id > 0 {
            volume_extruders.push(volume_extruder_id);
        }

        // push back filaments for features. The first level that stores the key wins, so an explicit
        // Default (0) on the volume or object keeps the part filament and does not fall back further.
        let mut push_feature_filament = |key: &str, global_filament: i32| {
            let vol_opt = self.config.option(key);
            let obj_opt = object.config.option(key);
            let filament = match (vol_opt, obj_opt) {
                (Some(v), _) => opt_get_int(v),
                (None, Some(o)) => opt_get_int(o),
                (None, None) => global_filament,
            };
            if filament > 0 {
                volume_extruders.push(filament);
            }
        };
        push_feature_filament("wall_filament", global_features.wall_filament);
        push_feature_filament("solid_infill_filament", global_features.solid_infill_filament);
        push_feature_filament("sparse_infill_filament", global_features.sparse_infill_filament);

        volume_extruders
    }

    /// Model.cpp:3468-3481 `update_extruder_count(size_t extruder_count)` of the volume
    /// `volume_idx` of `object` (its parent): the volume's own paint and extruder are clamped.
    pub fn update_extruder_count(object: &mut ModelObject, volume_idx: usize, extruder_count: usize) {
        let used_extruders = object.volumes[volume_idx].get_extruders(object, &GlobalFeatureFilaments::default());
        for &extruder_id in &used_extruders {
            if extruder_id as i64 > extruder_count as i64 {
                let v = &mut object.volumes[volume_idx];
                let mesh = v.m_mesh.clone().expect("ModelVolume::mesh(): no mesh");
                v.mmu_segmentation_facets.set_enforcer_block_type_limit(
                    &mesh,
                    EnforcerBlockerType(extruder_count as i8),
                    EnforcerBlockerType::NONE,
                    EnforcerBlockerType::NONE,
                );
                break;
            }
        }
        if object.volumes[volume_idx].extruder_id(object) as i64 > extruder_count as i64 {
            object.volumes[volume_idx].config.erase("extruder");
        }
    }

    /// Model.cpp:3483-3511 `update_extruder_count_when_delete_filament(extruder_count, filament_id,
    /// replace_filament_id, filament_is_mixed)` of the volume `volume_idx` of `object`.
    pub fn update_extruder_count_when_delete_filament(
        object: &mut ModelObject,
        volume_idx: usize,
        extruder_count: usize,
        filament_id: usize,
        replace_filament_id: i32,
        filament_is_mixed: &[u8],
    ) {
        // Global feature filaments are remapped by Plater::on_filaments_delete, not here.
        let used_extruders = object.volumes[volume_idx].get_extruders(object, &GlobalFeatureFilaments::default());
        for &extruder_id in &used_extruders {
            if extruder_id as i64 >= filament_id as i64 {
                let v = &mut object.volumes[volume_idx];
                let mesh = v.m_mesh.clone().expect("ModelVolume::mesh(): no mesh");
                v.mmu_segmentation_facets.set_enforcer_block_type_limit(
                    &mesh,
                    EnforcerBlockerType(extruder_count as i8),
                    EnforcerBlockerType(filament_id as i8),
                    EnforcerBlockerType(replace_filament_id as i8),
                );
                break;
            }
        }
        let eid = object.volumes[volume_idx].extruder_id(object) as usize;
        // Judge out-of-range against the post-remap id.
        let mut remapped = eid;
        if eid == filament_id {
            remapped = if replace_filament_id > 0 { replace_filament_id as usize } else { 1 };
        } else if eid > filament_id {
            remapped = eid - 1;
        }
        if remapped > extruder_count {
            // filament_is_mixed is the pre-delete snapshot; index it with the ORIGINAL eid (1-based).
            let is_mixed = !filament_is_mixed.is_empty() && eid >= 1 && (eid - 1) < filament_is_mixed.len() && filament_is_mixed[eid - 1] != 0;
            if !is_mixed {
                object.volumes[volume_idx].config.erase("extruder");
            }
        }
    }

    /// Model.cpp:3513-3529 `center_geometry_after_creation(bool update_source_offset)`: translates
    /// the mesh and the convex hull so that the origin of their vertices is the center of this
    /// volume's bounding box. Only just after creation, before the mesh is shared.
    pub fn center_geometry_after_creation(&mut self, update_source_offset: bool) {
        let shift = from_bb(&self.mesh().bounding_box().center());
        if !eigen_is_approx_vec3(&shift, &Vec3d::zeros()) {
            let d = Vec3f::new(-(shift[0] as f32), -(shift[1] as f32), -(shift[2] as f32));
            if let Some(m) = self.m_mesh.as_mut() {
                let m = Arc::make_mut(m);
                m.translate(&d);
                m.set_init_shift(&shift);
            }
            if let Some(h) = self.m_convex_hull.as_mut() {
                Arc::make_mut(h).translate(&d);
            }
            self.translate(&shift);
        }

        if update_source_offset {
            self.source.mesh_offset = shift;
        }
    }

    /// Model.cpp:3531-3535
    pub fn calculate_convex_hull(&mut self) {
        self.m_convex_hull = Some(Arc::new(self.mesh().convex_hull_3d()));
    }

    /// Model.cpp:3538-3566 `calculate_convex_hull_2d(const Geometry::Transformation &)`
    fn calculate_convex_hull_2d(&self, transformation: &Transformation) {
        let hull = match self.m_convex_hull.as_deref() {
            Some(h) => h,
            None => return,
        };
        let its = &hull.its;
        if its.vertices.is_empty() {
            return;
        }

        let mut new_trans = *transformation;
        new_trans.reset_offset();
        let new_matrix = new_trans.matrix();

        let mut pts = Vec::with_capacity(its.vertices.len());
        // Using the shared vertices should be a bit quicker than using the STL faces.
        for v in &its.vertices {
            let p = eigen_transform_point(&new_matrix, &Vec3d::new(v.x as f64, v.y as f64, v.z as f64));
            pts.push(crate::Point::new(coord_scale(p.x), coord_scale(p.y)));
        }
        let cached = crate::geometry::convex_hull_points(pts);
        *self.m_cached_2d_polygon.borrow_mut() = cached.clone();

        let mut hull_2d = cached;
        // MultiPoint::translate(double x, double y): Point(coord_t(x), coord_t(y))
        hull_2d.translate(crate::Point::new(coord_scale(transformation.get_offset_axis(Axis::X)), coord_scale(transformation.get_offset_axis(Axis::Y))));
        *self.m_convex_hull_2d.borrow_mut() = hull_2d;
    }

    /// Model.cpp:3568-3616 `get_convex_hull_2d(const Transform3d &trafo_instance)`
    pub fn get_convex_hull_2d(&self, trafo_instance: &Transform3d) -> Polygon {
        let new_matrix = eigen_mul_affine(trafo_instance, &self.m_transformation.matrix());

        let need_recompute = |old_transform: &Transformation, new_transform: &Transformation| -> bool {
            let old_rotation = old_transform.get_rotation();
            let new_rotation = new_transform.get_rotation();
            let old_mirror = old_transform.get_mirror();
            let new_mirror = new_transform.get_mirror();
            let old_scaling = old_transform.get_scaling_factor();
            let new_scaling = new_transform.get_scaling_factor();
            old_scaling != new_scaling || old_rotation != new_rotation || old_mirror != new_mirror
        };

        let valid = self.m_convex_hull_2d.borrow().is_valid();
        if new_matrix != self.m_cached_trans_matrix.get() || !valid {
            let new_trans = Transformation::from_transform(new_matrix);
            let old_trans = Transformation::from_transform(self.m_cached_trans_matrix.get());

            if need_recompute(&old_trans, &new_trans) || !valid {
                // need to update
                self.calculate_convex_hull_2d(&new_trans);
            } else {
                let mut hull_2d = self.m_cached_2d_polygon.borrow().clone();
                hull_2d.translate(crate::Point::new(coord_scale(new_trans.get_offset_axis(Axis::X)), coord_scale(new_trans.get_offset_axis(Axis::Y))));
                *self.m_convex_hull_2d.borrow_mut() = hull_2d;
            }
            self.m_cached_trans_matrix.set(new_matrix);
        }

        self.m_convex_hull_2d.borrow().clone()
    }

    /// Model.hpp:1110-1113
    pub fn invalidate_convex_hull_2d(&mut self) {
        self.m_convex_hull_2d.borrow_mut().clear();
    }

    /// Model.hpp:1107
    pub fn set_convex_hull_shared_ptr(&mut self, hull: Option<Arc<ModelMesh>>) {
        self.m_convex_hull = hull;
    }

    /// Model.hpp:1104
    pub fn get_convex_hull_shared_ptr(&self) -> Option<&Arc<ModelMesh>> {
        self.m_convex_hull.as_ref()
    }

    /// Model.hpp:1122
    pub fn get_transformation(&self) -> &Transformation {
        &self.m_transformation
    }

    /// Model.cpp:3618-3620 `set_transformation(const Geometry::Transformation &)`
    pub fn set_transformation(&mut self, transformation: &Transformation) {
        self.m_transformation = *transformation;
    }

    /// Model.cpp:3622-3624 `set_transformation(const Transform3d &trafo)`
    pub fn set_transformation_matrix(&mut self, trafo: &Transform3d) {
        self.m_transformation.set_from_transform(*trafo);
    }

    /// Model.cpp:3627-3632: per-volume assemble transformation.
    pub fn get_assemble_transformation(&self) -> &Transformation {
        if !self.m_assemble_initialized {
            return &self.m_transformation;
        }
        &self.m_assemble_transformation
    }

    /// Model.cpp:3634-3638
    pub fn set_assemble_transformation(&mut self, transformation: &Transformation) {
        self.m_assemble_initialized = true;
        self.m_assemble_transformation = *transformation;
    }

    /// Model.cpp:3640-3644
    pub fn set_assemble_from_transform(&mut self, transform: &Transform3d) {
        self.m_assemble_initialized = true;
        self.m_assemble_transformation.set_from_transform(*transform);
    }

    /// Model.cpp:3646-3650
    pub fn set_assemble_offset(&mut self, offset: &Vec3d) {
        self.m_assemble_initialized = true;
        self.m_assemble_transformation.set_offset(offset);
    }

    /// Model.hpp:1131
    pub fn is_assemble_initialized(&self) -> bool {
        self.m_assemble_initialized
    }

    /// Model.cpp:3652-3658
    pub fn get_repaired_errors_count(&self) -> i32 {
        let stats = &self.mesh().stats().repaired_errors;
        stats.degenerate_facets + stats.edges_fixed + stats.facets_removed + stats.facets_reversed + stats.backwards_edges
    }

    /// Model.cpp:3660-3663
    pub fn get_convex_hull(&self) -> &ModelMesh {
        self.m_convex_hull.as_deref().expect("ModelVolume::get_convex_hull(): no convex hull")
    }

    /// Model.cpp:3666-3682: refine the model part names.
    pub fn type_from_string(s: &str) -> ModelVolumeType {
        // New type (supporting the support enforcers & blockers)
        if s == "normal_part" {
            return ModelVolumeType::ModelPart;
        }
        if s == "negative_part" {
            return ModelVolumeType::NegativeVolume;
        }
        if s == "modifier_part" {
            return ModelVolumeType::ParameterModifier;
        }
        if s == "support_enforcer" {
            return ModelVolumeType::SupportEnforcer;
        }
        if s == "support_blocker" {
            return ModelVolumeType::SupportBlocker;
        }
        // Default value if invalud type string received.
        ModelVolumeType::ModelPart
    }

    /// Model.cpp:3685-3697
    pub fn type_to_string(t: ModelVolumeType) -> &'static str {
        match t {
            ModelVolumeType::ModelPart => "normal_part",
            ModelVolumeType::NegativeVolume => "negative_part",
            ModelVolumeType::ParameterModifier => "modifier_part",
            ModelVolumeType::SupportEnforcer => "support_enforcer",
            ModelVolumeType::SupportBlocker => "support_blocker",
            _ => "normal_part",
        }
    }

    /// Model.cpp:3702-3830 `split(unsigned int max_extruders, float scale_det)`: split the volume
    /// `volume_idx` of `object`, append the result after it; returns the number of volumes created
    /// from it.
    pub fn split(object: &mut ModelObject, volume_idx: usize, max_extruders: u32, scale_det: f32) -> usize {
        let _ = max_extruders;
        let mut ships: Vec<HashMap<i32, i32>> = Vec::new();
        let meshes = object.volumes[volume_idx].mesh().split_and_save_relationship(&mut ships, scale_det);
        if meshes.len() <= 1 {
            return 1;
        }
        if meshes.len() != ships.len() {
            return 1;
        }
        // splited volume should not be text object
        let mut idx: usize = 0;
        let mut ivolume = volume_idx;
        let this_name = object.volumes[volume_idx].name.clone();

        let offset = object.volumes[volume_idx].get_offset();
        // Capture the source volume's assembly-view transform up front.
        let src_assemble_initialized = object.volumes[volume_idx].is_assemble_initialized();
        let src_assemble_matrix = object.volumes[volume_idx].get_assemble_transformation().matrix();
        // BBS: also carry support/seam/fuzzy-skin painting across the split
        let face_count = object.volumes[volume_idx].m_mesh.as_ref().map(|m| m.its.indices.len()).unwrap_or(0);
        let mut tris_split_strs = Vec::with_capacity(face_count);
        let mut tris_sup_strs = Vec::with_capacity(face_count);
        let mut tris_seam_strs = Vec::with_capacity(face_count);
        let mut tris_fuzzy_strs = Vec::with_capacity(face_count);
        {
            let this = &object.volumes[volume_idx];
            for i in 0..face_count {
                tris_split_strs.push(this.mmu_segmentation_facets.get_triangle_as_string(i as i32));
                tris_sup_strs.push(this.supported_facets.get_triangle_as_string(i as i32));
                tris_seam_strs.push(this.seam_facets.get_triangle_as_string(i as i32));
                tris_fuzzy_strs.push(this.fuzzy_skin_facets.get_triangle_as_string(i as i32));
            }
        }
        let this_extruder_id = object.volumes[volume_idx].extruder_id(object);
        let this_is_text = object.volumes[volume_idx].is_text();
        let mut last_all_mesh_face_count = 0usize;
        // The volume being split is at `this_idx`; it does not move as pieces go after it.
        let this_idx = volume_idx;
        for mesh in meshes {
            if mesh.empty() {
                // Repair may have removed unconnected triangles, thus emptying the mesh.
                continue;
            }
            let cur_face_count = mesh.its.indices.len();
            let paint = |v: &mut ModelVolume, n: usize| {
                for i in 0..n {
                    if let Some(&index) = ships[idx].get(&(i as i32)) {
                        let index = index as usize;
                        if !tris_split_strs[index].is_empty() {
                            v.mmu_segmentation_facets.set_triangle_from_string(i as i32, &tris_split_strs[index]);
                        }
                        if !tris_sup_strs[index].is_empty() {
                            v.supported_facets.set_triangle_from_string(i as i32, &tris_sup_strs[index]);
                        }
                        if !tris_seam_strs[index].is_empty() {
                            v.seam_facets.set_triangle_from_string(i as i32, &tris_seam_strs[index]);
                        }
                        if !tris_fuzzy_strs[index].is_empty() {
                            v.fuzzy_skin_facets.set_triangle_from_string(i as i32, &tris_fuzzy_strs[index]);
                        }
                    }
                }
            };
            if idx == 0 {
                let this = &mut object.volumes[this_idx];
                this.set_mesh(mesh);
                this.calculate_convex_hull();
                this.invalidate_convex_hull_2d();
                // Assign a new unique ID, so that a new GLVolume will be generated.
                this.set_new_unique_id();
                this.ensure_part_guid(true);
                // reset the source to disable reload from disk
                this.source = ModelVolumeSource::default();

                // BBS: reset facet annotations
                this.mmu_segmentation_facets.reset();
                this.exterior_facets.reset();
                this.supported_facets.reset();
                this.fuzzy_skin_facets.reset();
                this.seam_facets.reset();
                paint(this, cur_face_count);
            } else {
                let mut new_mv = ModelVolume::new_copy_with_mesh(&object.volumes[this_idx], mesh);
                // new ModelVolume(object, *this, std::move(mesh))
                new_mv.set_model_object(Some(object.id()));
                let n = new_mv.mesh_ptr().map(|m| m.its.indices.len()).unwrap_or(0);
                paint(&mut new_mv, n);
                ivolume += 1;
                object.volumes.insert(ivolume, new_mv);
            }
            if ivolume < object.volumes.len() {
                let cur_vol = &mut object.volumes[ivolume];
                cur_vol.set_offset(&Vec3d::zeros());
                // center_geometry_after_creation() recenters the sub-mesh's local origin by its
                // bbox center; the inherited assemble transform is post-multiplied by that shift.
                let center_shift = from_bb(&cur_vol.mesh().bounding_box().center());
                cur_vol.center_geometry_after_creation(true);
                cur_vol.translate(&offset);
                if src_assemble_initialized {
                    let t = eigen_mul_affine(&src_assemble_matrix, &crate::geometry::geometry::translation_transform(&center_shift));
                    cur_vol.set_assemble_from_transform(&t);
                }
                cur_vol.name = format!("{}_{}", this_name, idx + 1);
                //BBS: always set the extruder id the same as original
                cur_vol.config.set("extruder", this_extruder_id);
                cur_vol.m_is_splittable.set(0);
                if this_is_text {
                    cur_vol.clear_text_info();
                }
            } else {
                continue;
            }
            idx += 1;
            last_all_mesh_face_count += cur_face_count;
        }
        let _ = last_all_mesh_face_count;

        // discard volumes for which the convex hull was not generated or is degenerate
        let mut i: usize = 0;
        while i < object.volumes.len() {
            let degenerate = match object.volumes[i].get_convex_hull_shared_ptr() {
                None => true,
                Some(hull) => hull.its.vertices.is_empty() || hull.its.indices.is_empty(),
            };
            if degenerate {
                object.delete_volume(i);
                idx = idx.wrapping_sub(1);
                i = i.wrapping_sub(1);
            }
            i = i.wrapping_add(1);
        }

        idx
    }

    /// Model.cpp:3832-3835
    pub fn translate(&mut self, displacement: &Vec3d) {
        let offset = self.get_offset() + displacement;
        self.set_offset(&offset);
    }

    /// Model.cpp:3837-3840
    pub fn scale(&mut self, scaling_factors: &Vec3d) {
        let sf = self.get_scaling_factor().component_mul(scaling_factors);
        self.set_scaling_factor(&sf);
    }

    /// Model.cpp:3855-3863 — not exterior_facets.
    pub fn assign_new_unique_ids_recursive(&mut self) {
        self.base.set_new_unique_id();
        self.config.set_new_unique_id();
        self.supported_facets.set_new_unique_id();
        self.fuzzy_skin_facets.set_new_unique_id();
        self.seam_facets.set_new_unique_id();
        self.mmu_segmentation_facets.set_new_unique_id();
    }

    /// Model.cpp:3865-3874 `rotate(double angle, Axis axis)`
    pub fn rotate_axis(&mut self, angle: f64, axis: Axis) {
        match axis {
            Axis::X => self.rotate(angle, &Vec3d::new(1.0, 0.0, 0.0)),
            Axis::Y => self.rotate(angle, &Vec3d::new(0.0, 1.0, 0.0)),
            Axis::Z => self.rotate(angle, &Vec3d::new(0.0, 0.0, 1.0)),
        }
    }

    /// Model.cpp:3876-3879 `rotate(double angle, const Vec3d& axis)`
    pub fn rotate(&mut self, angle: f64, axis: &Vec3d) {
        let m = eigen_angle_axis_rotation_matrix(angle, axis);
        let rotation = self.get_rotation() + crate::geometry::geometry::extract_euler_angles_from_matrix(&m);
        self.set_rotation(&rotation);
    }

    /// Model.cpp:3881-3892
    pub fn mirror(&mut self, axis: Axis) {
        let mut mirror = self.get_mirror();
        match axis {
            Axis::X => mirror[0] *= -1.0,
            Axis::Y => mirror[1] *= -1.0,
            Axis::Z => mirror[2] *= -1.0,
        }
        self.set_mirror(&mirror);
    }

    /// Model.cpp:3895-3903 `scale_geometry_after_creation(const Vec3f& versor)`: only before the
    /// meshes of this volume are shared.
    pub fn scale_geometry_after_creation(&mut self, versor: &Vec3f) {
        if let Some(m) = self.m_mesh.as_mut() {
            Arc::make_mut(m).scale(versor);
        }
        let hull_empty = self.m_convex_hull.as_ref().map(|h| h.empty()).unwrap_or(true);
        if hull_empty {
            //BBS: recompute the convex hull if it is null for previous too small
            self.calculate_convex_hull();
        } else if let Some(h) = self.m_convex_hull.as_mut() {
            Arc::make_mut(h).scale(versor);
        }
    }

    /// Model.cpp:3905-3915 `transform_this_mesh(const Transform3d &mesh_trafo, bool fix_left_handed)`
    pub fn transform_this_mesh(&mut self, mesh_trafo: &Transform3d, fix_left_handed: bool) {
        let mut mesh = self.mesh().clone();
        mesh.transform(mesh_trafo, fix_left_handed);
        self.set_mesh(mesh);
        let mut convex_hull = self.get_convex_hull().clone();
        convex_hull.transform(mesh_trafo, fix_left_handed);
        self.m_convex_hull = Some(Arc::new(convex_hull));
        // Let the rest of the application know that the geometry changed, so the meshes have to be reloaded.
        self.set_new_unique_id();
    }

    /// Model.cpp:3917-3927 `transform_this_mesh(const Matrix3d &matrix, bool fix_left_handed)`
    pub fn transform_this_mesh_matrix3(&mut self, matrix: &Matrix3d, fix_left_handed: bool) {
        let mut mesh = self.mesh().clone();
        mesh.transform_matrix3(matrix, fix_left_handed);
        self.set_mesh(mesh);
        let mut convex_hull = self.get_convex_hull().clone();
        convex_hull.transform_matrix3(matrix, fix_left_handed);
        self.m_convex_hull = Some(Arc::new(convex_hull));
        // Let the rest of the application know that the geometry changed, so the meshes have to be reloaded.
        self.set_new_unique_id();
    }

    /// Model.cpp:3929-3935
    pub fn convert_from_imperial_units(&mut self) {
        // assert(!self.source.is_converted_from_meters): compiled out in the reference build (-DNDEBUG)
        self.scale_geometry_after_creation(&Vec3f::new(25.4, 25.4, 25.4));
        self.set_offset(&Vec3d::zeros());
        self.source.is_converted_from_inches = true;
    }

    /// Model.cpp:3937-3943
    pub fn convert_from_meters(&mut self) {
        // assert(!self.source.is_converted_from_inches): compiled out in the reference build (-DNDEBUG)
        self.scale_geometry_after_creation(&Vec3f::new(1000.0, 1000.0, 1000.0));
        self.set_offset(&Vec3d::zeros());
        self.source.is_converted_from_meters = true;
    }

    /// Model.cpp:3967-3970
    pub fn get_matrix(&self, dont_translate: bool, dont_rotate: bool, dont_scale: bool, dont_mirror: bool) -> Transform3d {
        self.m_transformation.get_matrix(dont_translate, dont_rotate, dont_scale, dont_mirror)
    }

    /// Model.hpp:1166 with the default arguments.
    pub fn matrix(&self) -> Transform3d {
        self.m_transformation.matrix()
    }

    /// Model.hpp:1133
    pub fn get_offset(&self) -> Vec3d {
        self.m_transformation.get_offset()
    }
    /// Model.hpp:1134
    pub fn get_offset_axis(&self, axis: Axis) -> f64 {
        self.m_transformation.get_offset_axis(axis)
    }
    /// Model.hpp:1136
    pub fn set_offset(&mut self, offset: &Vec3d) {
        self.m_transformation.set_offset(offset);
    }
    /// Model.hpp:1137
    pub fn set_offset_axis(&mut self, axis: Axis, offset: f64) {
        self.m_transformation.set_offset_axis(axis, offset);
    }
    /// Model.hpp:1139
    pub fn get_rotation(&self) -> Vec3d {
        self.m_transformation.get_rotation()
    }
    /// Model.hpp:1140
    pub fn get_rotation_axis(&self, axis: Axis) -> f64 {
        self.m_transformation.get_rotation_axis(axis)
    }
    /// Model.hpp:1142
    pub fn set_rotation(&mut self, rotation: &Vec3d) {
        self.m_transformation.set_rotation(rotation);
    }
    /// Model.hpp:1144
    pub fn get_scaling_factor(&self) -> Vec3d {
        self.m_transformation.get_scaling_factor()
    }
    /// Model.hpp:1145
    pub fn get_scaling_factor_axis(&self, axis: Axis) -> f64 {
        self.m_transformation.get_scaling_factor_axis(axis)
    }
    /// Model.hpp:1147
    pub fn set_scaling_factor(&mut self, scaling_factor: &Vec3d) {
        self.m_transformation.set_scaling_factor(scaling_factor);
    }
    /// Model.hpp:1148
    pub fn set_scaling_factor_axis(&mut self, axis: Axis, scaling_factor: f64) {
        self.m_transformation.set_scaling_factor_axis(axis, scaling_factor);
    }
    /// Model.hpp:1150
    pub fn get_mirror(&self) -> Vec3d {
        self.m_transformation.get_mirror()
    }
    /// Model.hpp:1151
    pub fn get_mirror_axis(&self, axis: Axis) -> f64 {
        self.m_transformation.get_mirror_axis(axis)
    }
    /// Model.hpp:1152
    pub fn is_left_handed(&self) -> bool {
        self.m_transformation.is_left_handed()
    }
    /// Model.hpp:1154
    pub fn set_mirror(&mut self, mirror: &Vec3d) {
        self.m_transformation.set_mirror(mirror);
    }
    /// Model.hpp:1155
    pub fn set_mirror_axis(&mut self, axis: Axis, mirror: f64) {
        self.m_transformation.set_mirror_axis(axis, mirror);
    }
}

/// A random version-4 UUID as text (`boost::uuids::to_string(random_generator()())`).
fn random_uuid_string() -> String {
    use std::collections::hash_map::RandomState;
    use std::hash::{BuildHasher, Hasher};
    let mut bytes = [0u8; 16];
    for chunk in 0..2 {
        let mut h = RandomState::new().build_hasher();
        h.write_u64(chunk as u64);
        let v = h.finish().to_le_bytes();
        bytes[chunk * 8..chunk * 8 + 8].copy_from_slice(&v);
    }
    bytes[6] = (bytes[6] & 0x0f) | 0x40;
    bytes[8] = (bytes[8] & 0x3f) | 0x80;
    let hex: Vec<String> = bytes.iter().map(|b| format!("{:02x}", b)).collect();
    format!(
        "{}-{}-{}-{}-{}",
        hex[0..4].concat(),
        hex[4..6].concat(),
        hex[6..8].concat(),
        hex[8..10].concat(),
        hex[10..16].concat()
    )
}

/// Model.hpp:1434-1437
pub fn model_volumes_sort_by_id(model_volumes: &mut [ModelVolume]) {
    model_volumes.sort_by(|l, r| l.id().cmp(&r.id()));
}

/// Model.hpp:1439-1443: a volume of a list sorted by id.
pub fn model_volume_find_by_id(model_volumes: &[ModelVolume], id: ObjectID) -> Option<&ModelVolume> {
    let it = model_volumes.partition_point(|mv| mv.id() < id);
    if it != model_volumes.len() && model_volumes[it].id() == id {
        Some(&model_volumes[it])
    } else {
        None
    }
}

// =============================================================================================
// Model.hpp:1456-1600 — ModelInstance
// =============================================================================================

/// A single instance of a ModelObject: the affine transformation of an object.
/// Model.hpp:1456
///
/// `position`, `rotation_z` (degrees) and `scale` are the placement the loaders write and
/// [`ModelInstance::transform`] reads; the C++ members follow.
#[derive(Debug, Clone)]
pub struct ModelInstance {
    /// Instance position on the print bed (loaders).
    pub position: Point3F,
    /// Rotation around Z axis in degrees (loaders).
    pub rotation_z: f64,
    /// Scaling factors for X, Y, Z axes (loaders).
    pub scale: [f64; 3],
    /// Model.hpp:1469 — whether or not this instance is printable.
    pub printable: bool,
    base: ObjectBase,
    /// Model.hpp:1459
    m_transformation: Transformation,
    /// Model.hpp:1460
    m_assemble_transformation: Cell<Transformation>,
    /// Model.hpp:1461
    m_offset_to_assembly: Vec3d,
    /// Model.hpp:1462
    m_assemble_initialized: bool,
    /// Model.hpp:1463
    m_assemble_scalling_factor_dirty: Cell<bool>,
    /// Model.hpp:1467 — set by Print::validate() using ModelObject::check_instances_print_volume_state()
    pub print_volume_state: ModelInstanceEPrintVolumeState,
    /// Model.hpp:1470
    pub use_loaded_id_for_label: bool,
    /// Model.hpp:1471
    pub arrange_order: i32,
    /// Model.hpp:1472
    pub loaded_id: usize,
    /// Model.hpp:1573 `ModelObject* object`: the parent object, held as its ID (FIDELITY-NOTE
    /// (back pointers) at the top of this file).
    object: Option<ObjectID>,
}

/// The earlier name of [`ModelInstance`].
pub type Instance = ModelInstance;

/// ModelInstance methods for positioning and transformation.
impl ModelInstance {
    /// Model.hpp:1577 `explicit ModelInstance(ModelObject* object)`: a new unique ID.
    pub fn new() -> Self {
        Self {
            position: Point3F::new(0.0, 0.0, 0.0),
            rotation_z: 0.0,
            scale: [1.0, 1.0, 1.0],
            printable: true,
            base: ObjectBase::new(),
            m_transformation: Transformation::new(),
            m_assemble_transformation: Cell::new(Transformation::new()),
            m_offset_to_assembly: Vec3d::zeros(),
            m_assemble_initialized: false,
            m_assemble_scalling_factor_dirty: Cell::new(true),
            print_volume_state: ModelInstanceEPrintVolumeState::ModelInstancePvsInside,
            use_loaded_id_for_label: false,
            arrange_order: 0,
            loaded_id: 0,
            object: None,
        }
    }

    /// Model.hpp:1579-1586 `explicit ModelInstance(ModelObject *object, const ModelInstance &other)`:
    /// a new unique ID, the transformations and printable of `other`.
    pub fn new_from(other: &ModelInstance) -> Self {
        let mut i = Self::new();
        i.m_transformation = other.m_transformation;
        i.m_assemble_transformation.set(other.m_assemble_transformation.get());
        i.m_offset_to_assembly = other.m_offset_to_assembly;
        i.printable = other.printable;
        i.position = other.position;
        i.rotation_z = other.rotation_z;
        i.scale = other.scale;
        i
    }

    /// Create an instance at a specific position (loaders).
    pub fn at(position: Point3F) -> Self {
        let mut i = Self::new();
        i.position = position;
        i
    }

    /// Set the Z rotation and return self for chaining (loaders).
    pub fn with_rotation(mut self, degrees: f64) -> Self {
        self.rotation_z = degrees;
        self
    }

    /// Set uniform scale and return self for chaining (loaders).
    pub fn with_scale(mut self, scale: f64) -> Self {
        self.scale = [scale, scale, scale];
        self
    }

    /// The transformation of `position`, `rotation_z` and `scale` (loaders).
    pub fn transform(&self) -> Transform3D {
        Transform3D::identity()
            .translate(self.position.x, self.position.y, self.position.z)
            .rotate_z(self.rotation_z.to_radians())
            .scale(self.scale[0], self.scale[1], self.scale[2])
    }

    pub fn id(&self) -> ObjectID {
        self.base.id()
    }

    /// ObjectBase::assign_new_unique_ids_recursive
    pub fn assign_new_unique_ids_recursive(&mut self) {
        self.base.set_new_unique_id();
    }

    /// Model.hpp:1482 `ModelObject* get_object() const { return this->object; }`: the ID of the
    /// parent object, `None` for `nullptr`.
    pub fn get_object(&self) -> Option<ObjectID> {
        self.object
    }

    /// Model.hpp:1569 `void set_model_object(ModelObject *model_object) { object = model_object; }`:
    /// `model_object` is the ID of the parent object, `None` for `nullptr`.
    pub fn set_model_object(&mut self, model_object: Option<ObjectID>) {
        self.object = model_object;
    }

    /// Model.hpp:1474-1480
    pub fn get_labeled_id(&self) -> usize {
        if self.use_loaded_id_for_label && (self.loaded_id > 0) {
            self.loaded_id
        } else {
            self.id().id
        }
    }

    /// Model.hpp:1484
    pub fn get_transformation(&self) -> &Transformation {
        &self.m_transformation
    }

    /// Model.cpp:3972-3976
    pub fn set_transformation(&mut self, transformation: &Transformation) {
        self.m_transformation = *transformation;
        self.m_assemble_scalling_factor_dirty.set(true);
    }

    /// Model.cpp:3978-3986
    pub fn get_assemble_transformation(&self) -> Transformation {
        if self.m_assemble_scalling_factor_dirty.get() {
            let mut t = self.m_assemble_transformation.get();
            t.set_scaling_factor(&self.m_transformation.get_scaling_factor());
            self.m_assemble_transformation.set(t);
            self.m_assemble_scalling_factor_dirty.set(false);
        }
        self.m_assemble_transformation.get()
    }

    /// Model.cpp:3988-3992
    pub fn set_assemble_transformation(&mut self, transformation: &Transformation) {
        self.m_assemble_initialized = true;
        self.m_assemble_transformation.set(*transformation);
    }

    /// Model.cpp:3994-3998
    pub fn set_assemble_from_transform(&mut self, transform: &Transform3d) {
        self.m_assemble_initialized = true;
        let mut t = self.m_assemble_transformation.get();
        t.set_from_transform(*transform);
        self.m_assemble_transformation.set(t);
    }

    /// Model.hpp:1490
    pub fn get_assemble_offset(&self) -> Vec3d {
        self.m_assemble_transformation.get().get_offset()
    }

    /// Model.cpp:4000-4004
    pub fn set_assemble_offset(&mut self, offset: &Vec3d) {
        self.m_assemble_initialized = true;
        let mut t = self.m_assemble_transformation.get();
        t.set_offset(offset);
        self.m_assemble_transformation.set(t);
    }

    /// Model.hpp:1493
    pub fn set_assemble_rotation(&mut self, rotation: &Vec3d) {
        let mut t = self.m_assemble_transformation.get();
        t.set_rotation(rotation);
        self.m_assemble_transformation.set(t);
    }

    /// Model.hpp:1494-1496
    pub fn rotate_assemble(&mut self, angle: f64, axis: &Vec3d) {
        let mut t = self.m_assemble_transformation.get();
        let m = eigen_angle_axis_rotation_matrix(angle, axis);
        let rotation = t.get_rotation() + crate::geometry::geometry::extract_euler_angles_from_matrix(&m);
        t.set_rotation(&rotation);
        self.m_assemble_transformation.set(t);
    }

    /// Model.hpp:1499
    pub fn set_offset_to_assembly(&mut self, offset: &Vec3d) {
        self.m_offset_to_assembly = *offset;
    }
    /// Model.hpp:1500
    pub fn get_offset_to_assembly(&self) -> Vec3d {
        self.m_offset_to_assembly
    }
    /// Model.hpp:1502
    pub fn get_offset(&self) -> Vec3d {
        self.m_transformation.get_offset()
    }
    /// Model.hpp:1503
    pub fn get_offset_axis(&self, axis: Axis) -> f64 {
        self.m_transformation.get_offset_axis(axis)
    }
    /// Model.hpp:1505
    pub fn set_offset(&mut self, offset: &Vec3d) {
        self.m_transformation.set_offset(offset);
    }
    /// Model.hpp:1506
    pub fn set_offset_axis(&mut self, axis: Axis, offset: f64) {
        self.m_transformation.set_offset_axis(axis, offset);
    }
    /// Model.hpp:1508
    pub fn get_rotation(&self) -> Vec3d {
        self.m_transformation.get_rotation()
    }
    /// Model.hpp:1509
    pub fn get_rotation_axis(&self, axis: Axis) -> f64 {
        self.m_transformation.get_rotation_axis(axis)
    }
    /// Model.hpp:1511
    pub fn set_rotation(&mut self, rotation: &Vec3d) {
        self.m_transformation.set_rotation(rotation);
    }

    /// Model.cpp:4023-4027 `rotate(Matrix3d rotation_matrix)`
    pub fn rotate(&mut self, rotation_matrix: &Matrix3d) {
        let new_rotation_mat = eigen_mul_affine(&affine_of(rotation_matrix), &self.m_transformation.get_rotation_matrix());
        self.m_transformation.set_rotation_matrix(&new_rotation_mat);
    }

    /// Model.hpp:1516
    pub fn get_scaling_factor(&self) -> Vec3d {
        self.m_transformation.get_scaling_factor()
    }
    /// Model.hpp:1517
    pub fn get_scaling_factor_axis(&self, axis: Axis) -> f64 {
        self.m_transformation.get_scaling_factor_axis(axis)
    }

    /// Model.cpp:4006-4010
    pub fn set_scaling_factor(&mut self, scaling_factor: &Vec3d) {
        self.m_transformation.set_scaling_factor(scaling_factor);
        self.m_assemble_scalling_factor_dirty.set(true);
    }

    /// Model.cpp:4012-4016
    pub fn set_scaling_factor_axis(&mut self, axis: Axis, scaling_factor: f64) {
        self.m_transformation.set_scaling_factor_axis(axis, scaling_factor);
        self.m_assemble_scalling_factor_dirty.set(true);
    }

    /// Model.hpp:1522
    pub fn get_mirror(&self) -> Vec3d {
        self.m_transformation.get_mirror()
    }
    /// Model.hpp:1523
    pub fn get_mirror_axis(&self, axis: Axis) -> f64 {
        self.m_transformation.get_mirror_axis(axis)
    }
    /// Model.hpp:1524
    pub fn is_left_handed(&self) -> bool {
        self.m_transformation.is_left_handed()
    }
    /// Model.hpp:1526
    pub fn set_mirror(&mut self, mirror: &Vec3d) {
        self.m_transformation.set_mirror(mirror);
    }
    /// Model.hpp:1527
    pub fn set_mirror_axis(&mut self, axis: Axis, mirror: f64) {
        self.m_transformation.set_mirror_axis(axis, mirror);
    }

    /// Model.cpp:4018-4021 — to be called on an external mesh.
    pub fn transform_mesh(&self, mesh: &mut ModelMesh, dont_translate: bool) {
        mesh.transform(&self.get_matrix(dont_translate, false, false, false), false);
    }

    /// Model.cpp:4029-4054 — a bounding box of a transformed external mesh.
    pub fn transform_mesh_bounding_box(&self, mesh: &ModelMesh, dont_translate: bool) -> BoundingBoxf3 {
        // Rotate around mesh origin.
        let mut copy = mesh.clone();
        copy.transform(&self.get_matrix(true, false, true, true), false);
        let mut bbox = copy.bounding_box();

        if !bbox.is_empty() {
            // Scale the bounding box along the three axes.
            let mut mn = from_bb(&bbox.min);
            let mut mx = from_bb(&bbox.max);
            for i in 0..3 {
                let axis = [Axis::X, Axis::Y, Axis::Z][i];
                if (self.get_scaling_factor_axis(axis) - 1.0).abs() > EPSILON {
                    mn[i] *= self.get_scaling_factor_axis(axis);
                    mx[i] *= self.get_scaling_factor_axis(axis);
                }
            }

            // Translate the bounding box.
            if !dont_translate {
                mn += self.get_offset();
                mx += self.get_offset();
            }
            bbox.min = to_bb(&mn);
            bbox.max = to_bb(&mx);
        }
        bbox
    }

    /// Model.cpp:4056-4059 — transform an external bounding box.
    pub fn transform_bounding_box(&self, bbox: &BoundingBoxf3, dont_translate: bool) -> BoundingBoxf3 {
        eigen_bbox_transformed(bbox, &self.get_matrix(dont_translate, false, false, false))
    }

    /// Model.cpp:4061-4063
    pub fn transform_bounding_box_in_assembly_view(&self, bbox: &BoundingBoxf3, dont_translate: bool) -> BoundingBoxf3 {
        let _ = dont_translate;
        eigen_bbox_transformed(bbox, &self.get_assemble_transformation().matrix())
    }

    /// Model.cpp:4065-4069: world AABB of one part in assemble space.
    pub fn transform_volume_bounding_box_in_assembly_view(&self, volume: &ModelVolume) -> BoundingBoxf3 {
        let world = eigen_mul_affine(&self.get_assemble_transformation().matrix(), &volume.get_assemble_transformation().matrix());
        eigen_bbox_transformed(&volume.mesh().bounding_box(), &world)
    }

    /// Model.cpp:4071-4074 — transform an external vector.
    pub fn transform_vector(&self, v: &Vec3d, dont_translate: bool) -> Vec3d {
        eigen_transform_point(&self.get_matrix(dont_translate, false, false, false), v)
    }

    /// Model.cpp:4076-4082 — rotates and scales an external polygon, does not translate it.
    pub fn transform_polygon(&self, polygon: &mut Polygon) {
        // CHECK_ME -> Is the following correct or it should take in account all three rotations ?
        // MultiPoint::rotate(double angle) -> rotate(cos(angle), sin(angle)) (MultiPoint.cpp:27-35)
        let angle = self.get_rotation_axis(Axis::Z);
        let (c, s) = (angle.cos(), angle.sin());
        for pt in &mut polygon.points {
            let cur_x = pt.x as f64;
            let cur_y = pt.y as f64;
            pt.x = c.mul_add(cur_x, -(s * cur_y)).round() as crate::Coord;
            pt.y = c.mul_add(cur_y, s * cur_x).round() as crate::Coord;
        }
        // CHECK_ME -> Is the following correct ?
        // MultiPoint::scale(double factor_x, double factor_y) (MultiPoint.cpp:12-19)
        let (fx, fy) = (self.get_scaling_factor_axis(Axis::X), self.get_scaling_factor_axis(Axis::Y));
        for pt in &mut polygon.points {
            pt.x = (pt.x as f64 * fx) as crate::Coord;
            pt.y = (pt.y as f64 * fy) as crate::Coord;
        }
    }

    /// Model.cpp:4084-4087
    pub fn get_matrix(&self, dont_translate: bool, dont_rotate: bool, dont_scale: bool, dont_mirror: bool) -> Transform3d {
        self.m_transformation.get_matrix(dont_translate, dont_rotate, dont_scale, dont_mirror)
    }

    /// Model.hpp:1544 with the default arguments.
    pub fn matrix(&self) -> Transform3d {
        self.m_transformation.matrix()
    }

    /// Model.hpp:1545: `object` is the parent.
    pub fn is_printable(&self, object: &ModelObject) -> bool {
        object.printable && self.printable && (self.print_volume_state == ModelInstanceEPrintVolumeState::ModelInstancePvsInside)
    }

    /// Model.hpp:1546
    pub fn is_assemble_initialized(&self) -> bool {
        self.m_assemble_initialized
    }

    /// Model.cpp:4439-4455 `get_auto_brim_width(double deltaT, double adhension)`: `object` is
    /// the parent, `print_speed_map` and `extruder_params` are Model's static tables.
    pub fn get_auto_brim_width_with(
        &self,
        object: &ModelObject,
        delta_t: f64,
        adhension: f64,
        print_speed_map: &GlobalSpeedMap,
        extruder_params: &BTreeMap<usize, ExtruderParams>,
    ) -> f64 {
        let _ = delta_t;
        let raw_bbox = object.raw_mesh_bounding_box();
        let max_speed = Model::find_max_speed(object, print_speed_map);

        let bbox_size = from_bb(&self.transform_bounding_box(&raw_bbox, false).size());
        let height_to_area = std_max(
            bbox_size[2] / (bbox_size[0] * bbox_size[0] * bbox_size[1]),
            bbox_size[2] / (bbox_size[1] * bbox_size[1] * bbox_size[0]),
        );
        let thermal_length = bbox_size[0].mul_add(bbox_size[0], bbox_size[1] * bbox_size[1]).sqrt();
        let thermal_length_ref = Model::get_thermal_length_volumes(object, extruder_params);

        let mut brim_width = adhension
            * std_min(
                std_min(
                    std_max(
                        height_to_area * 200.0 * max_speed / 200.0,
                        thermal_length * 8.0 / thermal_length_ref * std_min(bbox_size[2], 30.0) / 30.0,
                    ),
                    20.0,
                ),
                1.5 * thermal_length,
            );
        // small brims are omitted
        if brim_width < 5.0 && brim_width < 1.5 * thermal_length {
            brim_width = 0.0;
        }
        brim_width
    }

    /// Model.cpp:4458-4472: the instance's 2D convex hull, `object` is the parent.
    ///
    /// FIDELITY-NOTE: the C++ keeps the result in `ModelInstance::convex_hull`, which nothing else
    /// reads; it is returned without being kept.
    pub fn convex_hull_2d(&self, object: &ModelObject) -> Polygon {
        // this logic is not working right now, as moving instance doesn't update convex_hull
        let trafo_instance = self.get_matrix(false, false, false, false);
        object.convex_hull_2d(&trafo_instance)
    }

    /// Model.cpp:4475-4478 — see [`ModelInstance::convex_hull_2d`]: nothing is kept.
    pub fn invalidate_convex_hull_2d(&mut self) {}

    /// Model.cpp:4521-4528 `get_auto_brim_width()`: returns 0 (the rest of the C++ body is after
    /// its `return 0.;`).
    pub fn get_auto_brim_width(&self) -> f64 {
        0.0
    }

    /// Model.cpp:4590-4597 — apply the arrange result to the instance `instance_idx` of `object`
    /// (the parent, whose bounding box is invalidated).
    pub fn apply_arrange_result(object: &mut ModelObject, instance_idx: usize, offs: &Vec2d, rotation: f64) {
        let this = &mut object.instances[instance_idx];
        // write the transformation data into the model instance
        let m = eigen_angle_axis_matrix3(rotation, &Vec3d::new(0.0, 0.0, 1.0));
        this.rotate(&m);
        this.set_offset_axis(Axis::X, offs.x * SCALING_FACTOR);
        this.set_offset_axis(Axis::Y, offs.y * SCALING_FACTOR);
        object.invalidate_bounding_box();
    }
}

/// Model.cpp:4480-4495 `getadhesionCoeff(const ModelVolumePtrs objectVolumes)`: adhesion
/// coefficients from the materials of the volumes of `object`; `extruder_params` is
/// `Model::extruderParamsMap`.
pub fn getadhesion_coeff(object: &ModelObject, extruder_params: &BTreeMap<usize, ExtruderParams>) -> f64 {
    let mut adhesion_coeff = 1.0;
    for model_volume in &object.volumes {
        if let Some(p) = extruder_params.get(&(model_volume.extruder_id(object) as usize)) {
            if p.material_name == "PETG" || p.material_name == "PCTG" {
                adhesion_coeff = 2.0;
            } else if p.material_name == "TPU" || p.material_name == "TPU-AMS" {
                adhesion_coeff = 0.5;
            }
        }
    }
    adhesion_coeff
}

/// Model.cpp:4497-4519 `getTemperatureFromExtruder(const ModelVolumePtrs objectVolumes)`: the bed
/// temperature of the first extruder of the volumes of `object` that has one.
pub fn get_temperature_from_extruder(object: &ModelObject, extruder_params: &BTreeMap<usize, ExtruderParams>) -> f64 {
    let mut extruders: Vec<usize> = Vec::new();
    for model_volume in &object.volumes {
        if model_volume.extruder_id(object) >= 0 {
            extruders.push(model_volume.extruder_id(object) as usize);
        }
    }

    let mut max_delta_temp = 0.0;
    for extruder_id in extruders {
        if let Some(p) = extruder_params.get(&extruder_id) {
            if p.bed_temp != 0 {
                max_delta_temp = std_max(max_delta_temp, p.bed_temp as f64);
                break;
            }
        }
    }
    max_delta_temp
}

/// Model.cpp:4182-4189 `get_real_filament_id(const unsigned char &id, std::string &result)`
pub fn get_real_filament_id(id: u8, result: &mut String) {
    if (id as usize) < CONST_FILAMENTS.len() {
        *result = CONST_FILAMENTS[id as usize].to_string();
    } else {
        // check error:CONST_FILAMENTS out of array
        *result = String::new(); //error
    }
}

impl ModelInstance {
    /// Model.cpp:4530-4588 `get_arrange_polygon(void *arrange_polygon, const DynamicPrintConfig &config_global)`:
    /// `object` is the parent, `extruder_params` is `Model::extruderParamsMap`.
    pub fn get_arrange_polygon(
        &self,
        object: &ModelObject,
        ret: &mut crate::model_arrange::ArrangePolygon,
        config_global: &DynamicPrintConfig,
        extruder_params: &BTreeMap<usize, ExtruderParams>,
    ) {
        let mut trafo_instance = *self.get_transformation();

        trafo_instance.set_offset(&Vec3d::new(0.0, 0.0, self.get_offset_axis(Axis::Z)));

        let p = object.convex_hull_2d(&trafo_instance.matrix());

        ret.poly.contour = p;
        ret.translation = crate::Point::new(coord_scale(self.get_offset_axis(Axis::X)), coord_scale(self.get_offset_axis(Axis::Y)));
        ret.rotation = 0.0;

        // BBS: add materials related information
        let get_filament_name = |id: i32| -> String {
            extruder_params.get(&(id as usize)).map(|p| p.material_name.clone()).unwrap_or_else(|| "PLA".to_string())
        };
        for volume in &object.volumes {
            if volume.is_model_part() {
                let ve = volume.get_extruders(object, &GlobalFeatureFilaments::from_full_config(config_global));
                for id in ve {
                    ret.extrude_id_filament_types.entry(id).or_insert_with(|| get_filament_name(id));
                }
            }
        }

        // get per-object support extruders
        let op = object.get_config_value(config_global, "enable_support");
        let is_support_enabled = op.map(|o| o.get_bool().unwrap_or(false)).unwrap_or(false);
        if is_support_enabled {
            let op1 = object.get_config_value(config_global, "support_filament").map(opt_get_int);
            let op2 = object.get_config_value(config_global, "support_interface_filament").map(opt_get_int);
            // id==0 means follow previous material, so need not be recorded
            if let Some(extruder_id) = op1 {
                if extruder_id > 0 {
                    ret.extrude_id_filament_types.entry(extruder_id).or_insert_with(|| get_filament_name(extruder_id));
                }
            }
            if let Some(extruder_id) = op2 {
                if extruder_id > 0 {
                    ret.extrude_id_filament_types.entry(extruder_id).or_insert_with(|| get_filament_name(extruder_id));
                }
            }
        }

        if ret.extrude_id_filament_types.is_empty() {
            // the default extruder
            ret.extrude_id_filament_types.entry(1).or_insert_with(|| get_filament_name(1));
        }
    }
}

/// Eigen `AngleAxisd(angle, axis).toRotationMatrix()` ([`crate::geometry::geometry::eigen::AngleAxisd`]).
pub fn eigen_angle_axis_matrix3(angle: f64, axis: &Vec3d) -> Matrix3d {
    crate::geometry::geometry::eigen::AngleAxisd::new(angle, *axis).to_rotation_matrix()
}

/// Default implementation for ModelInstance - creates instance at origin.
impl Default for ModelInstance {
    fn default() -> Self {
        Self::new()
    }
}

// =============================================================================================
// Model.hpp:1603-1631 — ModelWipeTower
// =============================================================================================

/// Model.hpp:1603 `class ModelWipeTower final : public ObjectBase`
#[derive(Debug, Clone)]
pub struct ModelWipeTower {
    base: ObjectBase,
    /// Model.hpp:1607
    pub positions: Vec<Vec2d>,
    /// Model.hpp:1608
    pub rotation: f64,
}

impl ModelWipeTower {
    /// Model.hpp:1617 — default constructor assigns a unique ID.
    pub fn new() -> Self {
        Self { base: ObjectBase::new(), positions: Vec::new(), rotation: 0.0 }
    }
    pub fn id(&self) -> ObjectID {
        self.base.id()
    }
}

// =============================================================================================
// Model.hpp:1656-1697 — profile, design and model info
// =============================================================================================

/// Model.hpp:1656-1664
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ModelProfileInfo {
    pub profile_tile: String,
    pub profile_cover: String,
    pub profile_description: String,
    pub profile_user_id: String,
    pub profile_user_name: String,
}

/// Model.hpp:1667-1673
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ModelDesignInfo {
    pub designer_id: String,
    pub designer: String,
    pub designer_user_id: String,
}

/// Model.hpp:1676-1697
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ModelInfo {
    pub cover_file: String,
    pub license: String,
    pub description: String,
    pub copyright: String,
    pub model_name: String,
    pub origin: String,
    pub metadata_items: BTreeMap<String, String>,
}

impl ModelInfo {
    /// Model.hpp:1688-1696
    pub fn load(&mut self, info: &ModelInfo) {
        self.cover_file = info.cover_file.clone();
        self.license = info.license.clone();
        self.description = info.description.clone();
        self.copyright = info.copyright.clone();
        self.model_name = info.model_name.clone();
        self.origin = info.origin.clone();
        self.metadata_items = info.metadata_items.clone();
    }
}

// =============================================================================================
// Model.hpp:1704-1962 — Model
// =============================================================================================

/// The print bed content. Model.hpp:1704
pub struct Model {
    /// Objects in this model.
    /// Model.hpp:1711
    pub objects: Vec<ModelObject>,
    /// Model metadata - contains title, author, and descriptive information (loaders).
    pub metadata: ModelMetadata,
    /// File path if loaded from file (loaders).
    pub file_path: Option<PathBuf>,
    base: ObjectBase,
    /// Model.hpp:1709 — materials are owned by a model and referenced by objects through
    /// t_model_material_id.
    pub materials: ModelMaterialMap,
    /// Model.hpp:1713
    pub step_import_path: String,
    /// Model.hpp:1716
    pub is_assembly_model: bool,
    /// Model.hpp:1718
    pub wipe_tower: ModelWipeTower,
    /// Model.hpp:1724-1730
    pub stl_design_id: String,
    pub design_id: String,
    pub stl_design_country: String,
    pub makerlab_region: String,
    pub makerlab_name: String,
    pub makerlab_id: String,
    /// Model.hpp:1732-1734
    pub design_info: Option<Arc<ModelDesignInfo>>,
    pub model_info: Option<Arc<ModelInfo>>,
    pub profile_info: Option<Arc<ModelProfileInfo>>,
    /// Model.hpp:1740-1743
    pub mk_name: String,
    pub mk_version: String,
    pub md_name: Vec<String>,
    pub md_value: Vec<String>,
    /// Model.hpp:1789
    pub curr_plate_index: i32,
    /// Model.hpp:1790
    pub plates_custom_gcodes: BTreeMap<i32, crate::custom_g_code::Info>,
    /// Model.hpp:1951-1954
    backup_path: String,
    need_backup: bool,
    object_backup_id_map: BTreeMap<usize, i32>,
    next_object_backup_id: i32,
    /// Model.hpp:1737 — textured mesh data for texture-to-painting import
    pub texture_mesh: Option<Arc<crate::texture_painting::TexturedMesh>>,
    /// Model.hpp:1956-1961
    m_assembly_tree_data: crate::format::assembly_tree_json::AssemblyTreeData,
    m_assembly_tree_json_str: String,
    m_assembly_steps_tree_data: crate::format::assembly_steps_json::AssemblyStepsTreeData,
    m_assembly_steps_json_str: String,
    m_assembly_model_json_str: String,
    m_loaded_assembly_steps_meta: crate::format::assembly_steps_json::AssemblyStepsFileMeta,
}

impl std::fmt::Debug for Model {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Model")
            .field("id", &self.id())
            .field("objects", &self.objects)
            .field("metadata", &self.metadata)
            .field("file_path", &self.file_path)
            .finish()
    }
}

impl Default for Model {
    /// Model.hpp:1800 — the default constructor assigns a new ID to the model (and to its wipe tower).
    fn default() -> Self {
        let base = ObjectBase::new();
        let wipe_tower = ModelWipeTower::new();
        Self {
            objects: Vec::new(),
            metadata: ModelMetadata::default(),
            file_path: None,
            base,
            materials: BTreeMap::new(),
            step_import_path: String::new(),
            is_assembly_model: false,
            wipe_tower,
            stl_design_id: String::new(),
            design_id: String::new(),
            stl_design_country: String::new(),
            makerlab_region: String::new(),
            makerlab_name: String::new(),
            makerlab_id: String::new(),
            design_info: None,
            model_info: None,
            profile_info: None,
            mk_name: String::new(),
            mk_version: String::new(),
            md_name: Vec::new(),
            md_value: Vec::new(),
            curr_plate_index: 0,
            plates_custom_gcodes: BTreeMap::new(),
            backup_path: String::new(),
            need_backup: false,
            object_backup_id_map: BTreeMap::new(),
            next_object_backup_id: 1,
            texture_mesh: None,
            m_assembly_tree_data: Default::default(),
            m_assembly_tree_json_str: String::new(),
            m_assembly_steps_tree_data: Default::default(),
            m_assembly_steps_json_str: String::new(),
            m_assembly_model_json_str: String::new(),
            m_loaded_assembly_steps_meta: Default::default(),
        }
    }
}

impl Clone for Model {
    /// Model.hpp:1805 `Model(const Model &rhs)`: the copy keeps the IDs (Model.cpp:65-126
    /// assign_copy); its `wipe_tower` member is default constructed, with a new ID.
    fn clone(&self) -> Self {
        let mut m = Model {
            objects: Vec::new(),
            metadata: self.metadata.clone(),
            file_path: self.file_path.clone(),
            base: ObjectBase::new_invalid(),
            materials: BTreeMap::new(),
            step_import_path: String::new(),
            is_assembly_model: false,
            wipe_tower: ModelWipeTower::new(),
            stl_design_id: String::new(),
            design_id: String::new(),
            stl_design_country: String::new(),
            makerlab_region: String::new(),
            makerlab_name: String::new(),
            makerlab_id: String::new(),
            design_info: None,
            model_info: None,
            profile_info: None,
            mk_name: String::new(),
            mk_version: String::new(),
            md_name: Vec::new(),
            md_value: Vec::new(),
            curr_plate_index: 0,
            plates_custom_gcodes: BTreeMap::new(),
            backup_path: String::new(),
            need_backup: false,
            object_backup_id_map: BTreeMap::new(),
            next_object_backup_id: 1,
            texture_mesh: None,
            m_assembly_tree_data: Default::default(),
            m_assembly_tree_json_str: String::new(),
            m_assembly_steps_tree_data: Default::default(),
            m_assembly_steps_json_str: String::new(),
            m_assembly_model_json_str: String::new(),
            m_loaded_assembly_steps_meta: Default::default(),
        };
        m.assign_copy(self);
        m
    }
}

/// Implementation of Model methods.
impl Model {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn id(&self) -> ObjectID {
        self.base.id()
    }

    /// Model.cpp:65-126 `assign_copy(const Model &rhs)`: the IDs copied.
    pub fn assign_copy(&mut self, rhs: &Model) -> &mut Self {
        self.base.copy_id(&rhs.base);
        let this = Some(self.id());
        // copy materials
        self.clear_materials();
        self.materials = rhs.materials.clone();
        for m in self.materials.values_mut() {
            // Copy including the ID and m_model.
            m.set_model(this);
        }
        // copy objects
        self.clear_objects();
        self.objects.reserve(rhs.objects.len());
        for model_object in &rhs.objects {
            // Copy including the ID, leave ID set to invalid (zero).
            let mut mo = model_object.clone();
            mo.set_model(this);
            self.objects.push(mo);
        }

        // copy custom code per height
        self.plates_custom_gcodes = rhs.plates_custom_gcodes.clone();
        self.curr_plate_index = rhs.curr_plate_index;

        // BBS: for design info
        self.design_info = rhs.design_info.clone();
        self.model_info = rhs.model_info.clone();
        self.design_id = rhs.design_id.clone();
        self.stl_design_id = rhs.stl_design_id.clone();
        self.stl_design_country = rhs.stl_design_country.clone();
        self.makerlab_region = rhs.makerlab_region.clone();
        self.makerlab_name = rhs.makerlab_name.clone();
        self.makerlab_id = rhs.makerlab_id.clone();
        self.profile_info = rhs.profile_info.clone();

        self.mk_name = rhs.mk_name.clone();
        self.mk_version = rhs.mk_version.clone();
        self.md_name = rhs.md_name.clone();
        self.md_value = rhs.md_value.clone();

        self.step_import_path = rhs.step_import_path.clone();
        self.m_assembly_tree_data = rhs.m_assembly_tree_data.clone();
        self.m_assembly_tree_json_str = rhs.m_assembly_tree_json_str.clone();
        self.m_assembly_steps_tree_data = rhs.m_assembly_steps_tree_data.clone();
        self.m_assembly_steps_json_str = rhs.m_assembly_steps_json_str.clone();
        self.m_assembly_model_json_str = rhs.m_assembly_model_json_str.clone();
        self.m_loaded_assembly_steps_meta = rhs.m_loaded_assembly_steps_meta;
        self.texture_mesh = rhs.texture_mesh.clone();
        self
    }

    /// Model.cpp:185-192
    pub fn assign_new_unique_ids_recursive(&mut self) {
        let old_id = self.id();
        self.base.set_new_unique_id();
        // FIDELITY-NOTE (back pointers): the children's handles follow the new ID.
        self.relink_children_from(old_id);
        for m in self.materials.values_mut() {
            m.assign_new_unique_ids_recursive();
        }
        for model_object in &mut self.objects {
            model_object.assign_new_unique_ids_recursive();
        }
    }

    /// The handles of the materials and objects that pointed at this model under `old_id` are
    /// re-pointed at its current ID (a C++ pointer does not change when its target takes a new
    /// ID; FIDELITY-NOTE (back pointers) at the top of this file).
    fn relink_children_from(&mut self, old_id: ObjectID) {
        let this = Some(self.id());
        for m in self.materials.values_mut() {
            if m.get_model() == Some(old_id) {
                m.set_model(this);
            }
        }
        for model_object in &mut self.objects {
            if model_object.get_model() == Some(old_id) {
                model_object.set_model(this);
            }
        }
    }

    /// Model.cpp:194-205 `update_links_bottom_up_recursive()`: point every material and object at
    /// this model, and every instance and volume at its object.
    pub fn update_links_bottom_up_recursive(&mut self) {
        let this = Some(self.id());
        for kvp in self.materials.values_mut() {
            kvp.set_model(this);
        }
        for model_object in &mut self.objects {
            model_object.set_model(this);
            let object = Some(model_object.id());
            for model_instance in &mut model_object.instances {
                model_instance.set_model_object(object);
            }
            for model_volume in &mut model_object.volumes {
                model_volume.set_model_object(object);
            }
        }
    }

    /// Model.cpp:207-215 — drop assembly tree / steps / a_model JSON.
    pub fn clear_assembly_artifacts(&mut self) {
        self.m_assembly_tree_data.clear();
        self.m_assembly_tree_json_str.clear();
        self.m_assembly_steps_tree_data.clear();
        self.m_assembly_steps_json_str.clear();
        self.m_assembly_model_json_str.clear();
        self.m_loaded_assembly_steps_meta = Default::default();
    }

    /// Model.cpp:226-282 `read_from_step(input_file, options, stepFn, stepIsUtf8Fn, step_mesh_fn,
    /// linear_defletion, angle_defletion, is_split_compound, open_shell_warn_fn)`: a model of the
    /// solids of a STEP file. `Err` is the `Slic3r::RuntimeError` the C++ throws.
    ///
    /// FIDELITY-NOTE: the reading and meshing (`Step::load`, `Step::mesh`) run on OpenCascade, which
    /// `format/step.rs` does not bind: its `load()` returns `LOAD_ERROR` after the UTF-8 check, so
    /// this function ends at "Loading of a model file failed." for every file. The control flow
    /// after the load is the C++'s. `Step::get_unclosed_shells()` (STEP.hpp:134, filled by the OCCT
    /// reader) has no counterpart in `format/step.rs`; `open_shell_warn_fn` would be called with
    /// those shells after a successful load and is not called here.
    #[allow(clippy::too_many_arguments)]
    pub fn read_from_step(
        input_file: &str,
        options: crate::format::bbs_3mf::LoadStrategy,
        step_fn: Option<crate::format::step::ImportStepProgressFn>,
        _step_is_utf8_fn: Option<crate::format::step::StepIsUtf8Fn>,
        step_mesh_fn: Option<&mut dyn FnMut(&mut crate::format::step::Step, &mut f64, &mut f64, &mut bool) -> i32>,
        mut linear_defletion: f64,
        mut angle_defletion: f64,
        mut is_split_compound: bool,
        open_shell_warn_fn: Option<&dyn Fn(&[String])>,
    ) -> Result<Model, crate::exception::RuntimeError> {
        use crate::exception::RuntimeError;
        use crate::format::bbs_3mf::LoadStrategy;
        use crate::format::step::{Step, StepStatus};

        let mut model = Model::new();
        let mut is_cb_cancel = false;
        // Step step_file(input_file, stepFn): the stepIsUtf8Fn argument is not passed on.
        let mut step_file = Step::from_path(input_file, step_fn, None);
        let mut status = step_file.load();
        '_finished: {
            if status != StepStatus::LoadSuccess {
                break '_finished;
            }
            // if (open_shell_warn_fn && !step_file.get_unclosed_shells().empty())
            //     open_shell_warn_fn(step_file.get_unclosed_shells());
            let _ = open_shell_warn_fn;
            if let Some(step_mesh_fn) = step_mesh_fn {
                if step_mesh_fn(&mut step_file, &mut linear_defletion, &mut angle_defletion, &mut is_split_compound) == -1 {
                    status = StepStatus::Cancel;
                    break '_finished;
                }
            }

            status = step_file.mesh(&mut model, &mut is_cb_cancel, is_split_compound, linear_defletion, angle_defletion);
        }

        match status {
            StepStatus::Cancel => {
                let empty_model = Model::new();
                return Ok(empty_model);
            }
            StepStatus::LoadError => return Err(RuntimeError::new("Loading of a model file failed.".to_string())),
            StepStatus::MeshError => {
                return Err(RuntimeError::new("Meshing of a model file failed or no valid shape.".to_string()))
            }
            _ => {}
        }

        if model.objects.is_empty() {
            return Err(RuntimeError::new("The supplied file couldn't be read because it's empty".to_string()));
        }

        for o in &mut model.objects {
            o.input_file = input_file.to_string();
        }

        if options.contains(LoadStrategy::AddDefaultInstances) {
            model.add_default_instances();
        }

        Ok(model)
    }

    /// Model.cpp:494-579 `read_from_archive(input_file, config, config_substitutions, out_file_type,
    /// options, plate_data, project_presets, file_version, proFn, project, color_group_map,
    /// volume_color_data, has_textures, has_unit)`: a model of a project file (3MF or zipped AMF).
    /// `Err` is the exception the C++ lets propagate (`Slic3r::RuntimeError`, or one of the
    /// loaders'). `plate_data`, `project_presets` and `file_version` are references here:
    /// `load_bbs_3mf` dereferences them unconditionally (bbs_3mf.cpp:10004).
    ///
    /// FIDELITY-NOTE: the PrusaSlicer 3MF importer (`format/three_mf.rs` `load_3mf`) and the AMF
    /// importer (`format/amf.rs` `load_amf`) take the earlier placeholder `calib::DynamicPrintConfig`
    /// and the `bbs_3mf::ConfigSubstitutionContext` shim, which carry no options: on those two paths
    /// the model is read as the C++ reads it, the configuration they read is not merged into
    /// `config`. The BBS 3MF path (`load_bbs_3mf`) reads into `config` as the C++ does.
    #[allow(clippy::too_many_arguments)]
    pub fn read_from_archive(
        input_file: &str,
        config: &mut DynamicPrintConfig,
        config_substitutions: &mut crate::config::ConfigSubstitutionContext,
        out_file_type: &mut En3mfType,
        options: crate::format::bbs_3mf::LoadStrategy,
        plate_data: &mut crate::format::bbs_3mf::PlateDataPtrs,
        project_presets: &mut Vec<crate::preset::Preset>,
        file_version: &mut crate::semver::Semver,
        pro_fn: Option<&crate::format::bbs_3mf::Import3mfProgressFn>,
        project: Option<&mut crate::project_task::BBLProject>,
        color_group_map: Option<&mut BTreeMap<i32, Vec<String>>>,
        volume_color_data: Option<&mut crate::format::bbs_3mf::VolumeColorInfoMap>,
        has_textures: Option<&mut bool>,
        has_unit: Option<&mut bool>,
    ) -> Result<Model, crate::exception::Exception> {
        use crate::exception::{Exception, RuntimeError};
        use crate::format::bbs_3mf::{
            load_bbs_3mf, LoadStrategy, IMPORT_STAGE_ADD_INSTANCE, IMPORT_STAGE_CHECK_MODE_GCODE, IMPORT_STAGE_UPDATE_GCODE,
        };

        let iends_with = |s: &str, suffix: &str| s.to_lowercase().ends_with(&suffix.to_lowercase());

        let mut model = Model::new();

        let mut result = false;
        // bool is_bbl_3mf; (uninitialized; every path that reads it writes it first)
        let mut is_bbl_3mf = false;
        if iends_with(input_file, ".3mf") {
            let mut prusa_file_parser = crate::format::three_mf::PrusaFileParser::new();
            if prusa_file_parser.check_3mf_from_prusa(input_file) {
                // for Prusa 3mf
                let mut prusa_config = crate::calib::DynamicPrintConfig::default();
                let mut prusa_substitutions = crate::format::bbs_3mf::ConfigSubstitutionContext::new();
                result = crate::format::three_mf::load_3mf(input_file, &mut prusa_config, &mut prusa_substitutions, &mut model, true)
                    .map_err(|e| Exception::new(e.to_string()))?;
                *out_file_type = En3mfType::FromPrusa;
                // PrusaSlicer always writes the unit, and load_3mf() converts by it.
                if let Some(has_unit) = has_unit {
                    *has_unit = true;
                }
            } else {
                // BBS: add part plate related logic
                // BBS: backup & restore
                result = load_bbs_3mf(
                    input_file,
                    config,
                    config_substitutions,
                    &mut model,
                    plate_data,
                    project_presets,
                    Some(&mut is_bbl_3mf),
                    file_version,
                    pro_fn,
                    options,
                    project,
                    0,
                    color_group_map,
                    volume_color_data,
                    has_textures,
                    has_unit,
                )
                .map_err(|e| Exception::new(e.0))?;
            }
        } else if iends_with(input_file, ".zip.amf") {
            let mut amf_config = crate::calib::DynamicPrintConfig::default();
            let mut amf_substitutions = crate::format::bbs_3mf::ConfigSubstitutionContext::new();
            result = crate::format::amf::load_amf(input_file, Some(&mut amf_config), Some(&mut amf_substitutions), &mut model, Some(&mut is_bbl_3mf));
        } else {
            return Err(RuntimeError::new("Unknown file format. Input file must have .3mf or .zip.amf extension.".to_string()).into());
        }

        if *out_file_type != En3mfType::FromPrusa {
            *out_file_type = if is_bbl_3mf { En3mfType::FromBBS } else { En3mfType::FromOther };
        }

        if !result {
            return Err(RuntimeError::new("Loading of a model file failed.".to_string()).into());
        }

        for o in &mut model.objects {
            o.input_file = input_file.to_string();
        }

        let mut cb_cancel = false;
        if options.contains(LoadStrategy::AddDefaultInstances) {
            model.add_default_instances();
            log::info!("import 3mf IMPORT_STAGE_ADD_INSTANCE\n");
            if let Some(pro_fn) = pro_fn {
                pro_fn(IMPORT_STAGE_ADD_INSTANCE, 0, 1, &mut cb_cancel);
                if cb_cancel {
                    return Err(RuntimeError::new("Canceled".to_string()).into());
                }
            }
        }

        log::info!("import 3mf IMPORT_STAGE_UPDATE_GCODE\n");
        if let Some(pro_fn) = pro_fn {
            pro_fn(IMPORT_STAGE_UPDATE_GCODE, 0, 1, &mut cb_cancel);
            if cb_cancel {
                return Err(RuntimeError::new("Canceled".to_string()).into());
            }
        }

        //BBS
        for plate_gcodes in model.plates_custom_gcodes.values_mut() {
            crate::custom_g_code::check_mode_for_custom_gcode_per_print_z(plate_gcodes);
        }

        log::info!("import 3mf IMPORT_STAGE_CHECK_MODE_GCODE\n");
        if let Some(pro_fn) = pro_fn {
            pro_fn(IMPORT_STAGE_CHECK_MODE_GCODE, 0, 1, &mut cb_cancel);
            if cb_cancel {
                return Err(RuntimeError::new("Canceled".to_string()).into());
            }
        }

        crate::print_config::config_def::handle_legacy_sla(config).map_err(|e| Exception::new(e.to_string()))?;

        Ok(model)
    }

    /// Insert a ModelObject into the objects collection and return its index (the loaders' form
    /// of Model.cpp:623 `add_object(const ModelObject &other)`, taking the object as it is).
    pub fn add_object(&mut self, mut object: ModelObject) -> usize {
        let id = self.objects.len();
        object.set_model(Some(self.id()));
        self.objects.push(object);
        id
    }

    /// Model.cpp:581-585 `ModelObject* Model::add_object()`
    pub fn add_object_new(&mut self) -> &mut ModelObject {
        // new ModelObject(this)
        let mut new_object = ModelObject::new_for_model();
        new_object.set_model(Some(self.id()));
        self.objects.push(new_object);
        self.objects.last_mut().unwrap()
    }

    /// Model.cpp:605-621 `add_object(const char *name, const char *path, TriangleMesh &&mesh)`
    pub fn add_object_with_mesh(&mut self, name: &str, path: &str, mesh: ModelMesh) -> &mut ModelObject {
        // new ModelObject(this)
        let mut new_object = ModelObject::new_for_model();
        new_object.set_model(Some(self.id()));
        self.objects.push(new_object);
        let object_idx = self.objects.len() as i32 - 1;
        let new_object = self.objects.last_mut().unwrap();
        new_object.name = name.to_string();
        new_object.input_file = path.to_string();
        let volume_idx = {
            let new_volume = new_object.add_volume_mesh(mesh, ModelVolumeType::ModelPart, true);
            new_volume.name = name.to_string();
            new_volume.source.input_file = path.to_string();
            new_volume.source.object_idx = object_idx;
            0
        };
        new_object.volumes.last_mut().unwrap().source.volume_idx = new_object.volumes.len() as i32 - 1 + volume_idx;
        // BBS: set default extruder id to 1
        if !new_object.config.has("extruder") || new_object.config.extruder() == 0 {
            new_object.config.set_key_value("extruder", Box::new(crate::config::ConfigOptionInt::with_value(1)));
        }
        new_object.invalidate_bounding_box();
        new_object
    }

    /// Model.cpp:623-644 `add_object(const ModelObject &other)`: a clone with new IDs.
    ///
    /// FIDELITY-NOTE: the backup bookkeeping (`need_backup`) has no file side here.
    pub fn add_object_clone(&mut self, other: &ModelObject) -> &mut ModelObject {
        let mut new_object = ModelObject::new_clone(other);
        new_object.set_model(Some(self.id()));
        // BBS: set default extruder id to 1
        if !new_object.config.has("extruder") || new_object.config.extruder() == 0 {
            new_object.config.set_key_value("extruder", Box::new(crate::config::ConfigOptionInt::with_value(1)));
        }
        self.objects.push(new_object);
        if self.need_backup {
            if let Some(backup) = self.object_backup_id_map.remove(&other.id().id) {
                let new_id = self.objects.last().unwrap().id().id;
                self.object_backup_id_map.insert(new_id, backup);
            }
        }
        self.objects.last_mut().unwrap()
    }

    /// Model.cpp:646-672 `set_assembly_pos(ModelObject *model_object)`: `object_idx` is the
    /// object's index in this model.
    pub fn set_assembly_pos(&mut self, object_idx: usize) {
        if self.objects.is_empty() {
            return;
        }
        let set_assembly_mo_offset = |model_object: &mut ModelObject, offset: &mut Vec3d, mo_box: &BoundingBoxf3| {
            for i in 0..model_object.instances.len() {
                if i >= 1 {
                    //along y axis
                    offset[1] += from_bb(&mo_box.size())[1] + 10.0;
                }
                model_object.instances[i].set_assemble_offset(offset);
            }
        };
        let mo_box = self.objects[object_idx].bounding_box_in_assembly_view();
        if self.objects.len() == 1 {
            let mut offset = Vec3d::new(0.0, 0.0, mo_box.size().z / 2.0f32 as f64);
            set_assembly_mo_offset(&mut self.objects[object_idx], &mut offset, &mo_box);
            return;
        }
        let cur_assemble_scene_box = self.bounding_box_in_assembly_view(Some(object_idx));
        if cur_assemble_scene_box.defined {
            let mut offset = from_bb(&cur_assemble_scene_box.center());
            offset[0] += (cur_assemble_scene_box.size().x / 2.0f32 as f64 + mo_box.size().x / 2.0f32 as f64) + 10.0; //fix space:10mm
            offset[2] = mo_box.size().z / 2.0f32 as f64; //on the ground
            set_assembly_mo_offset(&mut self.objects[object_idx], &mut offset, &mo_box);
        }
    }

    /// Model.cpp:674-681 `delete_object(size_t idx)`
    pub fn delete_object(&mut self, idx: usize) {
        self.objects.remove(idx);
    }

    /// Model.cpp:701-717 `delete_object(ObjectID id)`
    pub fn delete_object_by_id(&mut self, id: ObjectID) -> bool {
        if id.id != 0 {
            if let Some(idx) = self.objects.iter().position(|o| o.id() == id) {
                self.objects.remove(idx);
                return true;
            }
        }
        false
    }

    /// Model.cpp:719-732 — FIDELITY-NOTE: `step_import_tree_nodes` is not carried.
    pub fn clear_objects(&mut self) {
        self.objects.clear();
        self.step_import_path.clear();
        self.object_backup_id_map.clear();
        self.next_object_backup_id = 1;
        self.texture_mesh = None;
    }

    /// Model.cpp:751-755
    pub fn set_object_backup_id(&mut self, object: &ModelObject, uuid: i32) {
        self.object_backup_id_map.insert(object.id().id, uuid);
        if uuid >= self.next_object_backup_id {
            self.next_object_backup_id = uuid + 1;
        }
    }

    /// Model.cpp:757-764 — generate a new one if needed.
    pub fn get_object_backup_id(&mut self, object: &ModelObject) -> i32 {
        let next = &mut self.next_object_backup_id;
        *self.object_backup_id_map.entry(object.id().id).or_insert_with(|| {
            let v = *next;
            *next += 1;
            v
        })
    }

    /// Model.cpp:771-778
    pub fn delete_material(&mut self, material_id: &str) {
        self.materials.remove(material_id);
    }

    /// Model.cpp:780-785
    pub fn clear_materials(&mut self) {
        self.materials.clear();
    }

    /// Model.cpp:787-794 `add_material(t_model_material_id material_id)`
    pub fn add_material(&mut self, material_id: &str) -> &mut ModelMaterial {
        let created = !self.materials.contains_key(material_id);
        materials_add(&mut self.materials, material_id);
        let this = Some(self.id());
        let material = self.materials.get_mut(material_id).unwrap();
        if created {
            // new ModelMaterial(this)
            material.set_model(this);
        }
        material
    }

    /// Model.cpp:796-807 `add_material(material_id, const ModelMaterial &other)`
    pub fn add_material_copy(&mut self, material_id: &str, other: &ModelMaterial) -> &mut ModelMaterial {
        materials_add_copy(&mut self.materials, material_id, other);
        let this = Some(self.id());
        let material = self.materials.get_mut(material_id).unwrap();
        material.set_model(this);
        material
    }

    /// Model.hpp:1870-1873
    pub fn get_material(&self, material_id: &str) -> Option<&ModelMaterial> {
        self.materials.get(material_id)
    }

    /// Model.cpp:810-817 — makes sure all objects have at least one instance.
    pub fn add_default_instances(&mut self) -> bool {
        // apply a default position to all objects not having one
        for o in &mut self.objects {
            if o.instances.is_empty() {
                o.add_instance();
            }
        }
        true
    }

    /// Model.cpp:820-826 — the bounding box of the *transformed* instances.
    pub fn bounding_box(&self) -> BoundingBoxf3 {
        let mut bb = BoundingBoxf3::new();
        for o in &self.objects {
            bbox_merge(&mut bb, &o.bounding_box());
        }
        bb
    }

    /// Model.cpp:828-838 `bounding_box_in_assembly_view(ModelObject *model_object)`: the object
    /// skipped is given by its index.
    pub fn bounding_box_in_assembly_view(&self, model_object: Option<usize>) -> BoundingBoxf3 {
        let mut bb = BoundingBoxf3::new();
        for (i, o) in self.objects.iter().enumerate() {
            if model_object == Some(i) {
                continue;
            }
            bbox_merge(&mut bb, &o.bounding_box_in_assembly_view());
        }
        bb
    }

    /// Model.cpp:852-871
    pub fn center_instances_around_point(&mut self, point: &Vec2d) -> bool {
        let mut bb = BoundingBoxf3::new();
        for o in &self.objects {
            for i in 0..o.instances.len() {
                bbox_merge(&mut bb, &o.instance_bounding_box(i, false));
            }
        }

        let c = bb.center();
        let shift2 = Vec2d::new(point.x - c.x, point.y - c.y);
        if shift2[0].abs() < EPSILON && shift2[1].abs() < EPSILON {
            // No significant shift, don't do anything.
            return false;
        }

        let shift3 = Vec3d::new(shift2[0], shift2[1], 0.0);
        for o in &mut self.objects {
            for i in &mut o.instances {
                let offset = i.get_offset() + shift3;
                i.set_offset(&offset);
            }
            o.invalidate_bounding_box();
        }
        true
    }

    /// Model.hpp:1886 `translate(x, y, z)`
    pub fn translate(&mut self, x: f64, y: f64, z: f64) {
        for o in &mut self.objects {
            o.translate(x, y, z);
        }
    }

    /// Model.cpp:874-880 — flattens everything to a single mesh.
    pub fn mesh(&self) -> ModelMesh {
        let mut mesh = ModelMesh::new();
        for o in &self.objects {
            mesh.merge(&o.mesh());
        }
        mesh
    }

    /// Model.cpp:882-898 — errors out if there are several objects or none.
    pub fn duplicate_objects_grid(&mut self, x: usize, y: usize, dist: f64) -> Result<(), &'static str> {
        if self.objects.len() > 1 {
            return Err("Grid duplication is not supported with multiple objects");
        }
        if self.objects.is_empty() {
            return Err("No objects!");
        }

        let object = &mut self.objects[0];
        object.clear_instances();

        let ext_size = from_bb(&object.bounding_box().size()) + Vec3d::new(dist, dist, dist);

        for x_copy in 1..=x {
            for y_copy in 1..=y {
                let instance = object.add_instance();
                instance.set_offset(&Vec3d::new(ext_size[0] * (x_copy - 1) as f64, ext_size[1] * (y_copy - 1) as f64, 0.0));
            }
        }
        Ok(())
    }

    /// Model.cpp:900-917
    pub fn looks_like_multipart_object(&self) -> bool {
        if self.objects.len() <= 1 {
            return false;
        }
        let mut zmin = f64::MAX;
        for obj in &self.objects {
            if obj.volumes.len() > 1 || obj.config.keys().len() > 1 {
                return false;
            }

            let zmin_this = obj.get_min_z();
            if zmin == f64::MAX {
                zmin = zmin_this;
            } else if (zmin - zmin_this).abs() > EPSILON {
                // The Object don't share zmin.
                return true;
            }
        }
        false
    }

    /// Model.cpp:928-977
    pub fn convert_multipart_object(&mut self, max_extruders: u32) {
        let _ = max_extruders;
        // assert(self.objects.len() >= 2): compiled out in the reference build (-DNDEBUG)
        if self.objects.len() < 2 {
            return;
        }

        let mut object = ModelObject::new_for_model();
        object.input_file = self.objects[0].input_file.clone();
        object.name = std::path::Path::new(&self.objects[0].input_file)
            .file_stem()
            .map(|s| s.to_string_lossy().into_owned())
            .unwrap_or_default();
        object.origin_translation = self.objects[0].origin_translation;
        //FIXME copy the config etc?

        for o in &self.objects {
            for v in &o.volumes {
                // If there are more than one object, put all volumes together
                // Each object may contain any number of volumes and instances
                // The volumes transformations are relative to the object containing them...
                let mut trafo_volume = *v.get_transformation();
                // Revert the centering operation.
                trafo_volume.set_offset(&(trafo_volume.get_offset() - o.origin_translation));
                let mut counter = 1;
                let mut copy_volume = |new_v: &mut ModelVolume| {
                    new_v.name = if counter > 1 {
                        let s = format!("{}_{}", o.name, counter);
                        counter += 1;
                        s
                    } else {
                        o.name.clone()
                    };
                    //BBS: Use extruder priority: volumn > object > default
                    if v.config.option("extruder").is_some() {
                        new_v.config.set("extruder", v.config.extruder());
                    } else if o.config.option("extruder").is_some() {
                        new_v.config.set("extruder", o.config.extruder());
                    }
                };
                if o.instances.is_empty() {
                    let new_v = object.add_volume_copy(v, ModelVolumeType::Invalid);
                    copy_volume(new_v);
                    new_v.set_transformation(&trafo_volume);
                } else {
                    for i in &o.instances {
                        // ...so, transform everything to a common reference system (world)
                        let t = Transformation::from_transform(eigen_mul_affine(&i.get_transformation().matrix(), &trafo_volume.matrix()));
                        let new_v = object.add_volume_copy(v, ModelVolumeType::Invalid);
                        copy_volume(new_v);
                        new_v.set_transformation(&t);
                    }
                }
            }
        }

        self.clear_objects();
        self.objects.push(object);
    }

    /// Model.cpp:979 `volume_threshold_inches`, 981-1004
    pub fn looks_like_imperial_units(&self) -> bool {
        const VOLUME_THRESHOLD_INCHES: f64 = 8.0; // 9 = 2*2*2;
        if self.objects.is_empty() {
            return false;
        }

        for (oi, obj) in self.objects.iter().enumerate() {
            if (obj.get_object_stl_stats().volume as f64) < VOLUME_THRESHOLD_INCHES {
                if !obj.is_cut() {
                    return true;
                }
                let mut all_cut_parts_look_like_imperial_units = true;
                for (oj, obj_other) in self.objects.iter().enumerate() {
                    if oj == oi {
                        continue;
                    }
                    if obj_other.cut_id.is_equal(&obj.cut_id) && obj_other.get_object_stl_stats().volume as f64 >= VOLUME_THRESHOLD_INCHES {
                        all_cut_parts_look_like_imperial_units = false;
                        break;
                    }
                }
                if all_cut_parts_look_like_imperial_units {
                    return true;
                }
            }
        }
        false
    }

    /// Model.cpp:1006-1024
    pub fn convert_from_imperial_units(&mut self, only_small_volumes: bool) {
        const VOLUME_THRESHOLD_INCHES: f64 = 8.0;
        const IN_TO_MM: f32 = 25.4;
        for obj in &mut self.objects {
            if !only_small_volumes || (obj.get_object_stl_stats().volume as f64) < VOLUME_THRESHOLD_INCHES {
                obj.scale_mesh_after_creation(IN_TO_MM);
                for v in &mut obj.volumes {
                    // assert(!v.source.is_converted_from_meters): compiled out in the reference build (-DNDEBUG)
                    v.source.is_converted_from_inches = true;
                }
            }
        }
        if let Some(tm) = self.texture_mesh.as_mut() {
            for v in &mut Arc::make_mut(tm).vertices {
                v[0] *= IN_TO_MM;
                v[1] *= IN_TO_MM;
                v[2] *= IN_TO_MM;
            }
        }
    }

    /// Model.cpp:1026 `volume_threshold_meters`, 1028-1038
    pub fn looks_like_saved_in_meters(&self) -> bool {
        const VOLUME_THRESHOLD_METERS: f64 = 0.008; // 0.008 = 0.2*0.2*0.2
        if self.objects.is_empty() {
            return false;
        }

        for obj in &self.objects {
            if (obj.get_object_stl_stats().volume as f64) < VOLUME_THRESHOLD_METERS {
                return true;
            }
        }
        false
    }

    /// Model.cpp:1040-1059
    pub fn convert_from_meters(&mut self, only_small_volumes: bool) {
        const VOLUME_THRESHOLD_METERS: f64 = 0.008;
        const M_TO_MM: f64 = 1000.0;
        for obj in &mut self.objects {
            if !only_small_volumes || (obj.get_object_stl_stats().volume as f64) < VOLUME_THRESHOLD_METERS {
                obj.scale_mesh_after_creation(M_TO_MM as f32);
                for v in &mut obj.volumes {
                    // assert(!v.source.is_converted_from_inches): compiled out in the reference build (-DNDEBUG)
                    v.source.is_converted_from_meters = true;
                }
            }
        }
        if let Some(tm) = self.texture_mesh.as_mut() {
            let scale = M_TO_MM as f32;
            for v in &mut Arc::make_mut(tm).vertices {
                v[0] *= scale;
                v[1] *= scale;
                v[2] *= scale;
            }
        }
    }

    /// Model.cpp:1061 `zero_volume`, 1063-1075
    pub fn removed_objects_with_zero_volume(&mut self) -> i32 {
        const ZERO_VOLUME: f64 = 0.0000000001;
        if self.objects.is_empty() {
            return 0;
        }

        let mut removed = 0;
        let mut i = self.objects.len() as i32 - 1;
        while i >= 0 {
            if (self.objects[i as usize].get_object_stl_stats().volume as f64) < ZERO_VOLUME {
                self.delete_object(i as usize);
                removed += 1;
            }
            i -= 1;
        }
        removed
    }

    /// Model.cpp:1077-1094 — ensures that the min z of the model is not negative.
    pub fn adjust_min_z(&mut self) {
        if self.objects.is_empty() {
            return;
        }

        if self.bounding_box().min.z < 0.0 {
            for obj in &mut self.objects {
                let obj_min_z = obj.bounding_box().min.z;
                if obj_min_z < 0.0 {
                    obj.translate_instances(&Vec3d::new(0.0, 0.0, -obj_min_z));
                }
            }
        }
    }

    /// Model.cpp:1098-1113 — the first printable object's export name.
    pub fn propose_export_file_name_and_path(&self) -> String {
        let mut input_file = String::new();
        'end: for model_object in &self.objects {
            for model_instance in &model_object.instances {
                if model_instance.is_printable(model_object) {
                    input_file = model_object.get_export_filename();

                    if !input_file.is_empty() {
                        break 'end;
                    }
                    // Other instances will produce the same name, skip them.
                    break;
                }
            }
        }
        input_file
    }

    /// Model.cpp:1117-1120 `get_auxiliary_file_temp_path()`
    pub fn get_auxiliary_file_temp_path(&mut self) -> String {
        self.get_backup_path_sub("/Auxiliaries")
    }

    /// Model.cpp:1123-1189 `get_backup_path()`: the backup directory of this model, created (with
    /// `3D/Objects`, `Metadata` and `lock.txt`) when it is used first.
    ///
    /// FIDELITY-NOTE: the free-space check of the system temp directory (and the fall back to
    /// `data_dir()/tmp`) is not made: the crate has no file system space query.
    pub fn get_backup_path(&mut self) -> String {
        if self.backup_path.is_empty() {
            let pid = crate::utils::get_current_pid();
            let parent_path = std::path::PathBuf::from(crate::utils::temporary_dir());
            let now_time = chrono::Local::now();
            let buf = format!("/bamboo_model/{}#{}#{}", now_time.format("%a_%b_%d/%H_%M_%S"), pid, self.id().id);

            self.backup_path = format!("{}{}", parent_path.to_string_lossy(), buf);
            log::info!("model {:p}, id {}, backup_path empty, set to {}", self, self.id().id, self.backup_path);
            let temp_path = std::path::Path::new(&self.backup_path);
            if temp_path.exists() {
                log::info!("model {:p}, id {}, remove previous {}", self, self.id().id, self.backup_path);
                let _ = std::fs::remove_dir_all(temp_path);
            }
        }
        let temp_path = std::path::PathBuf::from(&self.backup_path);
        if !temp_path.exists() {
            log::info!("create /3D/Objects in {}", self.backup_path);
            if let Err(ex) = std::fs::create_dir_all(format!("{}/3D/Objects", self.backup_path)) {
                log::error!("Failed to create backup path{}: {}", self.backup_path, ex);
            }
            log::info!("create /Metadata in {}", self.backup_path);
            let _ = std::fs::create_dir_all(format!("{}/Metadata", self.backup_path));
            log::info!("create /lock.txt in {}", self.backup_path);
            let _ = crate::utils::save_string_file(
                std::path::Path::new(&format!("{}/lock.txt", self.backup_path)),
                &crate::utils::get_current_pid().to_string(),
            );
        }

        self.backup_path.clone()
    }

    /// Model.cpp:1191-1202 `remove_backup_path_if_exist()`: remove the backup directory (when it
    /// exists) and forget it.
    pub fn remove_backup_path_if_exist(&mut self) {
        if !self.backup_path.is_empty() {
            let temp_path = std::path::PathBuf::from(&self.backup_path);
            if temp_path.exists() {
                log::info!("model {:p}, id {} remove backup_path {}", self, self.id().id, self.backup_path);
                let _ = std::fs::remove_dir_all(&temp_path);
            }
            self.backup_path.clear();
        }
    }

    /// Model.cpp:1204-1216 `get_backup_path(const std::string &sub_path)`
    pub fn get_backup_path_sub(&mut self, sub_path: &str) -> String {
        let path = format!("{}/{}", self.get_backup_path(), sub_path);
        if !std::path::Path::new(&path).exists() {
            log::info!("create missing sub_path{}", path);
            if let Err(ex) = std::fs::create_dir_all(&path) {
                log::error!("Failed to create missing sub_path{}: {}", path, ex);
            }
        }
        path
    }

    /// Model.cpp:1218-1232 `set_backup_path` — FIDELITY-NOTE: no backup files are removed.
    pub fn set_backup_path(&mut self, path: &str) {
        if self.backup_path == path {
            return;
        }
        if "detach" == path {
            self.backup_path.clear();
            return;
        }
        self.backup_path = path.to_string();
    }

    /// Model.cpp:1269-1272
    pub fn set_need_backup(&mut self) {
        self.need_backup = true;
    }

    /// Model.hpp:1917
    pub fn is_need_backup(&self) -> bool {
        self.need_backup
    }

    /// Model.cpp:1274-1277 — the proposed path with the extension replaced (the new extension
    /// contains the initial dot).
    pub fn propose_export_file_name_and_path_ext(&self, new_extension: &str) -> String {
        let p = std::path::PathBuf::from(self.propose_export_file_name_and_path());
        let ext = new_extension.strip_prefix('.').unwrap_or(new_extension);
        p.with_extension(ext).to_string_lossy().into_owned()
    }

    /// Model.cpp:1279-1282
    pub fn is_fdm_support_painted(&self) -> bool {
        self.objects.iter().any(|mo| mo.is_fdm_support_painted())
    }

    /// Model.cpp:1284-1287
    pub fn is_seam_painted(&self) -> bool {
        self.objects.iter().any(|mo| mo.is_seam_painted())
    }

    /// Model.cpp:1289-1292
    pub fn is_mm_painted(&self) -> bool {
        self.objects.iter().any(|mo| mo.is_mm_painted())
    }

    /// Model.hpp:1745-1752
    pub fn set_designer(&mut self, designer: &str, designer_user_id: &str) {
        let mut info = self.design_info.as_deref().cloned().unwrap_or_default();
        info.designer = designer.to_string();
        //BBS tips: clean design user id when set designer
        info.designer_user_id = designer_user_id.to_string();
        self.design_info = Some(Arc::new(info));
    }

    /// Model.hpp:1754-1755
    pub fn get_assembly_tree_data(&self) -> &crate::format::assembly_tree_json::AssemblyTreeData {
        &self.m_assembly_tree_data
    }
    /// Model.hpp:1756
    pub fn set_assembly_tree_data(&mut self, data: crate::format::assembly_tree_json::AssemblyTreeData) {
        self.m_assembly_tree_data = data;
    }
    /// Model.hpp:1763-1764
    pub fn get_assembly_steps_tree_data(&self) -> &crate::format::assembly_steps_json::AssemblyStepsTreeData {
        &self.m_assembly_steps_tree_data
    }
    /// Model.hpp:1765
    pub fn set_assembly_steps_tree_data(&mut self, data: crate::format::assembly_steps_json::AssemblyStepsTreeData) {
        self.m_assembly_steps_tree_data = data;
    }
    /// Model.hpp:1779 — captured from assembly_step.json before qualified_assembly is stripped on load.
    pub fn note_loaded_assembly_steps_meta(&mut self, meta: crate::format::assembly_steps_json::AssemblyStepsFileMeta) {
        self.m_loaded_assembly_steps_meta = meta;
    }
    /// Model.hpp:1780
    pub fn loaded_assembly_steps_meta(&self) -> &crate::format::assembly_steps_json::AssemblyStepsFileMeta {
        &self.m_loaded_assembly_steps_meta
    }
    /// Model.hpp:1758
    pub fn get_assembly_tree_json_str(&self) -> &str {
        &self.m_assembly_tree_json_str
    }
    /// Model.hpp:1767
    pub fn get_assembly_steps_json_str(&self) -> &str {
        &self.m_assembly_steps_json_str
    }
    /// Model.hpp:1774
    pub fn get_assembly_model_json_str(&self) -> &str {
        &self.m_assembly_model_json_str
    }
    /// Model.hpp:1760
    pub fn set_assembly_tree_json_str(&mut self, json_str: String) {
        self.m_assembly_tree_json_str = json_str;
    }
    /// Model.hpp:1769
    pub fn set_assembly_steps_json_str(&mut self, json_str: String) {
        self.m_assembly_steps_json_str = json_str;
    }
    /// Model.hpp:1776
    pub fn set_assembly_model_json_str(&mut self, json_str: String) {
        self.m_assembly_model_json_str = json_str;
    }

    /// Model.hpp:1792-1797
    pub fn get_curr_plate_custom_gcodes(&self) -> crate::custom_g_code::Info {
        if let Some(info) = self.plates_custom_gcodes.get(&self.curr_plate_index) {
            return info.clone();
        }
        crate::custom_g_code::Info::default()
    }

    /// Model.hpp:1903
    pub fn print_info(&self) {
        for o in &self.objects {
            o.print_info();
        }
    }

    /// Model.cpp:4368-4403 `findMaxSpeed(const ModelObject* object)`: `print_speed_map` is
    /// `Model::printSpeedMap`.
    pub fn find_max_speed(object: &ModelObject, print_speed_map: &GlobalSpeedMap) -> f64 {
        let object_keys = object.config.keys();
        let mut obj_max_speed = -1.0;
        if object_keys.is_empty() {
            return print_speed_map.max_speed;
        }
        let mut perimeter_speed_obj = print_speed_map.perimeter_speed;
        let mut external_perimeter_speed_obj = print_speed_map.external_perimeter_speed;
        let mut infill_speed_obj = print_speed_map.infill_speed;
        let mut solid_infill_speed_obj = print_speed_map.solid_infill_speed;
        let mut top_solid_infill_speed_obj = print_speed_map.top_solid_infill_speed;
        let mut support_speed_obj = print_speed_map.support_speed;
        let mut small_perimeter_speed_obj = print_speed_map.small_perimeter_speed;
        let data = object.config.get();
        for object_key in &object_keys {
            // todo multi_extruders:
            if object_key == "inner_wall_speed" {
                perimeter_speed_obj = *data.opt_float_nullable(object_key, 0);
                external_perimeter_speed_obj = print_speed_map.external_perimeter_speed / print_speed_map.perimeter_speed * perimeter_speed_obj;
            }
            if object_key == "sparse_infill_speed" {
                infill_speed_obj = *data.opt_float_nullable(object_key, 0);
            }
            if object_key == "internal_solid_infill_speed" {
                solid_infill_speed_obj = *data.opt_float_nullable(object_key, 0);
            }
            if object_key == "top_surface_speed" {
                top_solid_infill_speed_obj = *data.opt_float_nullable(object_key, 0);
            }
            if object_key == "support_speed" {
                support_speed_obj = *data.opt_float_nullable(object_key, 0);
            }
            if object_key == "outer_wall_speed" {
                external_perimeter_speed_obj = *data.opt_float_nullable(object_key, 0);
            }
            if object_key == "small_perimeter_speed" {
                if let Some(o) = data.opt::<crate::config::ConfigOptionFloatsOrPercentsNullable>(object_key) {
                    small_perimeter_speed_obj = o.get_at(0).get_abs_value(external_perimeter_speed_obj);
                }
            }
        }
        obj_max_speed = std_max(
            perimeter_speed_obj,
            std_max(
                external_perimeter_speed_obj,
                std_max(
                    infill_speed_obj,
                    std_max(solid_infill_speed_obj, std_max(top_solid_infill_speed_obj, std_max(support_speed_obj, std_max(small_perimeter_speed_obj, obj_max_speed)))),
                ),
            ),
        );
        if obj_max_speed <= 0.0 {
            obj_max_speed = 250.0;
        }
        obj_max_speed
    }

    /// Model.cpp:4405-4424 `getThermalLength(const ModelVolume* modelVolumePtr)`: thermal length
    /// by the material of a volume; `object` is its parent, `extruder_params` is
    /// `Model::extruderParamsMap`.
    pub fn get_thermal_length(model_volume: &ModelVolume, object: &ModelObject, extruder_params: &BTreeMap<usize, ExtruderParams>) -> f64 {
        let mut thermal_length = 200.0;
        let aa = model_volume.extruder_id(object);
        if let Some(p) = extruder_params.get(&(aa as usize)) {
            if p.material_name == "ABS" || p.material_name == "PA-CF" || p.material_name == "PET-CF" {
                thermal_length = 100.0;
            }
            if p.material_name == "PC" {
                thermal_length = 40.0;
            }
            if p.material_name == "TPU" || p.material_name == "TPU-AMS" {
                thermal_length = 1000.0;
            }
        }
        thermal_length
    }

    /// Model.cpp:4426-4437 `getThermalLength(const std::vector<ModelVolume*> modelVolumePtrs)`:
    /// the shortest thermal length of the volumes of `object`.
    pub fn get_thermal_length_volumes(object: &ModelObject, extruder_params: &BTreeMap<usize, ExtruderParams>) -> f64 {
        let mut thermal_length = 1250.0;

        for model_volume in &object.volumes {
            // the thermal length of a group is decided by the volume with shortest thermal length
            thermal_length = std_min(thermal_length, Self::get_thermal_length(model_volume, object, extruder_params));
        }
        thermal_length
    }

    /// Model.cpp:4091-4148 `setPrintSpeedTable(const DynamicPrintConfig& config, const PrintConfig& print_config)`:
    /// set the print speed table and find the maximum speed. `print_speed_map` is
    /// `Model::printSpeedMap`; `bed_exclude_area` is `print_config.bed_exclude_area.values` (the
    /// static PrintConfig has no struct in the crate with that member; `g_code.rs` takes the
    /// option the same way).
    pub fn set_print_speed_table(config: &DynamicPrintConfig, bed_exclude_area: &[crate::geometry::Vec2d], print_speed_map: &mut GlobalSpeedMap) {
        let has = |key: &str| {
            use crate::config::ConfigOptionResolver;
            config.optptr(key).is_some()
        };
        //Slic3r::DynamicPrintConfig config = wxGetApp().preset_bundle->full_config();
        print_speed_map.max_speed = 0.0;

        // todo multi_extruders: the following parameters need get exact filament id
        if has("inner_wall_speed") {
            print_speed_map.perimeter_speed = *config.opt_float_nullable("inner_wall_speed", 0);
            if print_speed_map.perimeter_speed > print_speed_map.max_speed {
                print_speed_map.max_speed = print_speed_map.perimeter_speed;
            }
        }
        if has("outer_wall_speed") {
            print_speed_map.external_perimeter_speed = *config.opt_float_nullable("outer_wall_speed", 0);
            print_speed_map.max_speed = std_max(print_speed_map.max_speed, print_speed_map.external_perimeter_speed);
        }
        if has("sparse_infill_speed") {
            print_speed_map.infill_speed = *config.opt_float_nullable("sparse_infill_speed", 0);
            if print_speed_map.infill_speed > print_speed_map.max_speed {
                print_speed_map.max_speed = print_speed_map.infill_speed;
            }
        }
        if has("internal_solid_infill_speed") {
            print_speed_map.solid_infill_speed = *config.opt_float_nullable("internal_solid_infill_speed", 0);
            if print_speed_map.solid_infill_speed > print_speed_map.max_speed {
                print_speed_map.max_speed = print_speed_map.solid_infill_speed;
            }
        }
        if has("top_surface_speed") {
            print_speed_map.top_solid_infill_speed = *config.opt_float_nullable("top_surface_speed", 0);
            if print_speed_map.top_solid_infill_speed > print_speed_map.max_speed {
                print_speed_map.max_speed = print_speed_map.top_solid_infill_speed;
            }
        }
        if has("support_speed") {
            print_speed_map.support_speed = *config.opt_float_nullable("support_speed", 0);

            if print_speed_map.support_speed > print_speed_map.max_speed {
                print_speed_map.max_speed = print_speed_map.support_speed;
            }
        }

        //auto& print = wxGetApp().plater()->get_partplate_list().get_current_fff_print();
        //auto print_config = print.config();
        //printSpeedMap.bed_poly.points = get_bed_shape(*(wxGetApp().plater()->config()));
        print_speed_map.bed_poly.points = get_bed_shape(config, true);
        let excluse_area_points = bed_exclude_area;
        let mut exclude_polys: Vec<Polygon> = Vec::new();
        let mut exclude_poly = Polygon::new();
        for i in 0..excluse_area_points.len() {
            let pt = excluse_area_points[i];
            exclude_poly.points.push(crate::Point::new(coord_scale(pt.x), coord_scale(pt.y)));
            if i % 4 == 3 {
                // exclude areas are always rectangle
                exclude_polys.push(exclude_poly.clone());
                exclude_poly.points.clear();
            }
        }
        let available_bed = crate::clipper_utils::diff_polygons_raw_clib(&[print_speed_map.bed_poly.clone()], &exclude_polys);
        if !available_bed.is_empty() {
            print_speed_map.bed_poly = available_bed.into_iter().next().unwrap();
        } else {
            print_speed_map.bed_poly.points.clear();
        }
    }

    /// Model.cpp:4151-4180 `setExtruderParams(const DynamicPrintConfig &config, int filament_count)`:
    /// the temperature of the heat end and bed and the material of each extruder, as the new
    /// `Model::extruderParamsMap`.
    pub fn set_extruder_params(config: &DynamicPrintConfig, filament_count: i32) -> BTreeMap<usize, ExtruderParams> {
        let mut extruder_params_map = BTreeMap::new();
        let has = |key: &str| {
            use crate::config::ConfigOptionResolver;
            config.optptr(key).is_some()
        };
        for i in 0..filament_count.max(0) as u32 {
            let mut mat_name = String::new();
            // BBS
            let bed_temp = 35;
            let mut end_temp = 0.0f32 as f64;
            if has("filament_type") {
                mat_name = config.opt_string_at("filament_type", i).clone();
            }
            if has("nozzle_temperature") {
                end_temp = *config.opt_int_nullable("nozzle_temperature", i) as f64;
            }

            // FIXME: curr_bed_type is now a plate config rather than a global config.
            if i == 0 {
                extruder_params_map.entry(i as usize).or_insert_with(|| ExtruderParams {
                    material_name: mat_name.clone(),
                    bed_temp,
                    heat_end_temp: end_temp,
                });
            }
            extruder_params_map.entry(i as usize + 1).or_insert_with(|| ExtruderParams {
                material_name: mat_name.clone(),
                bed_temp,
                heat_end_temp: end_temp,
            });
        }
        extruder_params_map
    }

    /// Model.cpp:735-749 `collect_reusable_objects(std::vector<ObjectBase*>& objects)`: the IDs of
    /// the objects and their volumes, the volume IDs kept in `volume_ids`, the objects released.
    pub fn collect_reusable_objects(&mut self, objects: &mut Vec<ObjectID>) {
        for model_object in &mut self.objects {
            objects.push(model_object.id());
            for model_volume in &model_object.volumes {
                objects.push(model_volume.id());
            }
            let ids: Vec<ObjectID> = model_object.volumes.iter().map(|v| v.id()).collect();
            model_object.volume_ids.extend(ids);
            model_object.volumes.clear();
        }
        // we never own these objects
        self.objects.clear();
    }

    /// Model.cpp:1234-1266 `load_from(Model& model)` — FIDELITY-NOTE: the backup directory itself
    /// (`get_backup_path()`) is not created here; its path is moved.
    pub fn load_from(&mut self, model: &mut Model) {
        let path = std::mem::take(&mut model.backup_path);
        self.set_backup_path(&path);
        self.object_backup_id_map = model.object_backup_id_map.clone();
        self.next_object_backup_id = model.next_object_backup_id;
        self.design_info = model.design_info.clone();
        self.design_id = model.design_id.clone();
        self.stl_design_id = model.stl_design_id.clone();
        self.stl_design_country = model.stl_design_country.clone();
        self.makerlab_region = model.makerlab_region.clone();
        self.makerlab_name = model.makerlab_name.clone();
        self.makerlab_id = model.makerlab_id.clone();
        self.model_info = model.model_info.clone();
        self.profile_info = model.profile_info.clone();
        self.mk_name = model.mk_name.clone();
        self.mk_version = model.mk_version.clone();
        self.md_name = model.md_name.clone();
        self.md_value = model.md_value.clone();
        self.texture_mesh = model.texture_mesh.take();
        self.step_import_path = model.step_import_path.clone();
        self.m_assembly_tree_data = model.m_assembly_tree_data.clone();
        self.m_assembly_tree_json_str = model.m_assembly_tree_json_str.clone();
        self.m_assembly_steps_tree_data = model.m_assembly_steps_tree_data.clone();
        self.m_assembly_steps_json_str = model.m_assembly_steps_json_str.clone();
        self.m_assembly_model_json_str = model.m_assembly_model_json_str.clone();
        self.m_loaded_assembly_steps_meta = model.m_loaded_assembly_steps_meta;
        model.design_info = None;
        model.model_info = None;
        model.profile_info = None;
    }

    /// Model.cpp:840-850 `update_print_volume_state(const BuildVolume &build_volume)`: the
    /// number of printable instances.
    pub fn update_print_volume_state(&mut self, build_volume: &crate::build_volume::BuildVolume) -> u32 {
        let mut num_printable = 0;
        for model_object in &mut self.objects {
            num_printable += model_object.update_instances_print_volume_state(build_volume);
        }
        num_printable
    }

    /// Count total number of instances across all ModelObjects.
    pub fn total_instances(&self) -> usize {
        self.objects.iter().map(|o| o.instances.len()).sum()
    }

    /// Check whether the model contains no objects.
    pub fn is_empty(&self) -> bool {
        self.objects.is_empty()
    }

    /// Return the number of ModelObjects in this Model.
    pub fn object_count(&self) -> usize {
        self.objects.len()
    }

    /// Retrieve a reference to an object at the given index.
    pub fn get_object(&self, id: usize) -> Option<&ModelObject> {
        self.objects.get(id)
    }

    /// Retrieve a mutable reference to an object at the given index.
    pub fn get_object_mut(&mut self, id: usize) -> Option<&mut ModelObject> {
        self.objects.get_mut(id)
    }
}

// =============================================================================================
// Model.hpp:348-714 — ModelObject
// =============================================================================================

/// `t_layer_config_ranges`: `std::map<t_layer_height_range, ModelConfig>` (Slicing.hpp).
pub type TLayerConfigRanges = BTreeMap<(ordered_float::OrderedFloat<f64>, ordered_float::OrderedFloat<f64>), ModelConfig>;

#[derive(Debug, Clone)]
/// A printable object, possibly having multiple print volumes (each with its own set of
/// parameters and materials), and possibly having multiple modifier volumes.
/// Model.hpp:348
pub struct ModelObject {
    /// Model.hpp:351
    pub name: String,
    /// The merged mesh the loaders fill (C++ keeps the geometry in `volumes`).
    pub mesh: TriangleMesh,
    /// Model.hpp:357 — instances of this ModelObject, owned by it.
    pub instances: Vec<ModelInstance>,
    /// Model.hpp:362 — configuration parameters specific to a single ModelObject, overriding the
    /// global Slic3r settings.
    pub config: ModelConfigObject,
    /// Model.hpp:369 — whether or not this object is printable.
    pub printable: bool,
    base: ObjectBase,
    /// Model.hpp:353
    pub module_name: String,
    /// Model.hpp:354
    pub input_file: String,
    /// Model.hpp:360 — printable and modifier volumes, owned by this ModelObject.
    pub volumes: Vec<ModelVolume>,
    /// Model.hpp:364
    pub layer_config_ranges: TLayerConfigRanges,
    /// Model.hpp:367
    pub layer_height_profile: LayerHeightProfile,
    /// Model.hpp:382
    pub brim_points: Vec<crate::brim_ears_point::BrimPoint>,
    /// Model.hpp:388 — the total translation applied by center_around_origin().
    pub origin_translation: Vec3d,
    /// Model.hpp:391
    pub volume_ids: Vec<ObjectID>,
    /// Model.hpp:394
    pub cut_connectors: CutConnectors,
    /// Model.hpp:395
    pub cut_id: CutObjectBase,
    /// Model.hpp:662-668 — the bounding boxes, cached.
    m_bounding_box: Cell<BoundingBoxf3>,
    m_bounding_box_in_assembly_view: Cell<BoundingBoxf3>,
    m_bounding_box_valid: Cell<bool>,
    m_raw_bounding_box: Cell<BoundingBoxf3>,
    m_raw_bounding_box_valid: Cell<bool>,
    m_raw_mesh_bounding_box: Cell<BoundingBoxf3>,
    m_raw_mesh_bounding_box_valid: Cell<bool>,
    /// Model.hpp:659 `Model *m_model = nullptr;`: the parent model, held as its ID
    /// (FIDELITY-NOTE (back pointers) at the top of this file).
    m_model: Option<ObjectID>,
}

/// ModelObject methods for geometry and instance management.
impl ModelObject {
    /// An object of the loaders: one merged mesh and one default instance (the IDs as
    /// Model.hpp:574 `explicit ModelObject(Model* model)`, then the instance's).
    pub fn new(name: impl Into<String>, mesh: TriangleMesh) -> Self {
        let mut o = Self::new_for_model();
        o.name = name.into();
        o.mesh = mesh;
        o.instances = vec![ModelInstance::default()];
        o
    }

    /// Model.hpp:574-580 `explicit ModelObject(Model* model)`: new IDs for the object, its config
    /// and its layer height profile; no instances, no volumes.
    pub fn new_for_model() -> Self {
        let base = ObjectBase::new();
        let config = ModelConfigObject::new();
        let layer_height_profile = LayerHeightProfile::new();
        Self {
            name: String::new(),
            mesh: TriangleMesh::new(),
            instances: Vec::new(),
            config,
            printable: true,
            base,
            module_name: String::new(),
            input_file: String::new(),
            volumes: Vec::new(),
            layer_config_ranges: BTreeMap::new(),
            layer_height_profile,
            brim_points: Vec::new(),
            origin_translation: Vec3d::zeros(),
            volume_ids: Vec::new(),
            cut_connectors: Vec::new(),
            cut_id: CutObjectBase::new(),
            m_bounding_box: Cell::new(BoundingBoxf3::new()),
            m_bounding_box_in_assembly_view: Cell::new(BoundingBoxf3::new()),
            m_bounding_box_valid: Cell::new(false),
            m_raw_bounding_box: Cell::new(BoundingBoxf3::new()),
            m_raw_bounding_box_valid: Cell::new(false),
            m_raw_mesh_bounding_box: Cell::new(BoundingBoxf3::new()),
            m_raw_mesh_bounding_box_valid: Cell::new(false),
            m_model: None,
        }
    }

    /// Model.hpp:139-145 `new_clone(const ModelObject &rhs)`: a copy with new IDs
    /// (Model.cpp:1320-1363 assign_copy, then 1407-1415 assign_new_unique_ids_recursive).
    pub fn new_clone(rhs: &ModelObject) -> Self {
        let mut obj = rhs.clone();
        obj.assign_new_unique_ids_recursive();
        obj
    }

    /// Model.hpp:146-152 `make_clone(const ModelObject &rhs)`: a copy with new IDs, by value.
    pub fn make_clone(rhs: &ModelObject) -> ModelObject {
        // Default constructor assigning an invalid ID.
        let mut obj = rhs.clone();
        obj.assign_new_unique_ids_recursive();
        obj
    }

    pub fn id(&self) -> ObjectID {
        self.base.id()
    }

    /// Model.hpp:650-654
    fn set_new_unique_id(&mut self) {
        let old_id = self.id();
        self.base.set_new_unique_id();
        self.config.set_new_unique_id();
        self.layer_height_profile.set_new_unique_id();
        // FIDELITY-NOTE (back pointers): the children's handles follow the new ID.
        self.relink_children_from(old_id);
    }

    /// The handles of the volumes and instances that pointed at this object under `old_id` are
    /// re-pointed at its current ID (FIDELITY-NOTE (back pointers) at the top of this file).
    fn relink_children_from(&mut self, old_id: ObjectID) {
        let this = Some(self.id());
        for model_volume in &mut self.volumes {
            if model_volume.get_object() == Some(old_id) {
                model_volume.set_model_object(this);
            }
        }
        for model_instance in &mut self.instances {
            if model_instance.get_object() == Some(old_id) {
                model_instance.set_model_object(this);
            }
        }
    }

    /// Model.hpp:398 `Model* get_model()` (and 399, its const overload): the ID of the parent
    /// model, `None` for `nullptr`.
    pub fn get_model(&self) -> Option<ObjectID> {
        self.m_model
    }

    /// Model.hpp:673 `void set_model(Model *model) { m_model = model; }`: `model` is the ID of
    /// the parent model, `None` for `nullptr`.
    pub fn set_model(&mut self, model: Option<ObjectID>) {
        self.m_model = model;
    }

    /// Model.cpp:1407-1415
    pub fn assign_new_unique_ids_recursive(&mut self) {
        self.set_new_unique_id();
        for model_volume in &mut self.volumes {
            model_volume.assign_new_unique_ids_recursive();
        }
        for model_instance in &mut self.instances {
            model_instance.assign_new_unique_ids_recursive();
        }
        self.layer_height_profile.set_new_unique_id();
    }

    /// Model.cpp:1426 `get_backup_id()`: `model` is the parent.
    pub fn get_backup_id(&self, model: Option<&mut Model>) -> i32 {
        match model {
            Some(m) => m.get_object_backup_id(self),
            None => -1,
        }
    }

    /// Model.hpp:397
    pub fn const_volumes(&self) -> Vec<&ModelVolume> {
        self.volumes.iter().collect()
    }

    /// Model.cpp:1450-1461 `add_volume(const TriangleMesh &mesh, bool modify_to_center_geometry)`
    /// and 1463-1474 `add_volume(TriangleMesh &&mesh, ModelVolumeType type, bool modify_to_center_geometry)`.
    pub fn add_volume_mesh(&mut self, mesh: ModelMesh, type_: ModelVolumeType, modify_to_center_geometry: bool) -> &mut ModelVolume {
        let mut v = ModelVolume::new(mesh, type_);
        v.set_model_object(Some(self.id()));
        self.volumes.push(v);
        if modify_to_center_geometry {
            self.volumes.last_mut().unwrap().center_geometry_after_creation(true);
            self.invalidate_bounding_box();
        }
        self.volumes.last_mut().unwrap()
    }

    /// Model.cpp:1476-1491 `add_volume(const ModelVolume &other, ModelVolumeType type)`
    pub fn add_volume_copy(&mut self, other: &ModelVolume, type_: ModelVolumeType) -> &mut ModelVolume {
        let mut v = ModelVolume::new_copy_of(other);
        v.set_model_object(Some(self.id()));
        if type_ != ModelVolumeType::Invalid && v.type_() != type_ {
            v.set_type(type_);
        }

        v.cut_info = other.cut_info;

        self.volumes.push(v);
        // The volume should already be centered at this point of time when copying shared pointers of the triangle mesh and convex hull.
        self.volumes.last_mut().unwrap()
    }

    /// Model.cpp:1493-1502 `add_volume(const ModelVolume &other, TriangleMesh &&mesh)`
    pub fn add_volume(&mut self, other: &ModelVolume, mesh: ModelMesh) -> &mut ModelVolume {
        let mut v = ModelVolume::new_copy_with_mesh(other, mesh);
        v.set_model_object(Some(self.id()));
        self.volumes.push(v);
        self.volumes.last_mut().unwrap().center_geometry_after_creation(true);
        self.invalidate_bounding_box();
        self.volumes.last_mut().unwrap()
    }

    /// Model.cpp:1504-1516 `add_volume_with_shared_mesh(const ModelVolume &other, ModelVolumeType type)`
    pub fn add_volume_with_shared_mesh(&mut self, other: &ModelVolume, type_: ModelVolumeType) -> &mut ModelVolume {
        let mesh = other.m_mesh.clone().unwrap_or_else(|| Arc::new(ModelMesh::new()));
        let mut v = ModelVolume::new_shared(mesh, ModelVolumeType::ModelPart);
        v.set_model_object(Some(self.id()));
        if type_ != ModelVolumeType::Invalid && v.type_() != type_ {
            v.set_type(type_);
        }
        self.volumes.push(v);
        // The volume should already be centered at this point of time when copying shared pointers of the triangle mesh and convex hull.
        self.volumes.last_mut().unwrap()
    }

    /// Model.cpp:1518-1555
    pub fn delete_volume(&mut self, idx: usize) {
        self.volumes.remove(idx);

        if !self.brim_points.is_empty() {
            // remove the brim ear points of the deleted part and shift the volume_idx of the later parts
            self.brim_points.retain(|pt| pt.volume_idx != idx as i32);
            for pt in &mut self.brim_points {
                if pt.volume_idx > idx as i32 {
                    pt.volume_idx -= 1;
                }
            }
        }

        if self.volumes.len() == 1 {
            // only one volume left
            // we need to collapse the volume transform into the instances transforms because now when selecting this volume
            // it will be seen as a single full instance ans so its volume transform may be ignored
            let v_t = self.volumes[0].get_transformation().matrix();
            for inst in &mut self.instances {
                let t = Transformation::from_transform(eigen_mul_affine(&inst.get_transformation().matrix(), &v_t));
                inst.set_transformation(&t);
            }
            let t = Transformation::new();
            let v = &mut self.volumes[0];
            v.set_transformation(&t);
            v.set_new_unique_id();
        }

        self.invalidate_bounding_box();
    }

    /// Model.cpp:1557-1565
    pub fn clear_volumes(&mut self) {
        self.volumes.clear();
        self.invalidate_bounding_box();
    }

    /// Model.cpp:1567-1570
    pub fn is_fdm_support_painted(&self) -> bool {
        self.volumes.iter().any(|mv| mv.is_fdm_support_painted())
    }

    /// Model.cpp:1572-1575
    pub fn is_seam_painted(&self) -> bool {
        self.volumes.iter().any(|mv| mv.is_seam_painted())
    }

    /// Model.cpp:1577-1580
    pub fn is_mm_painted(&self) -> bool {
        self.volumes.iter().any(|mv| mv.is_mm_painted())
    }

    /// Model.cpp:1582-1585
    pub fn is_fuzzy_skin_painted(&self) -> bool {
        self.volumes.iter().any(|mv| mv.is_fuzzy_skin_facets_painted())
    }

    /// Model.hpp:417
    pub fn is_multiparts(&self) -> bool {
        self.volumes.len() > 1
    }

    /// Model.hpp:428-429
    pub fn has_custom_layering(&self) -> bool {
        !self.layer_config_ranges.is_empty() || !self.layer_height_profile.empty()
    }

    /// Model.cpp:1587-1601: "Model Part, Negative Volume, Modifier, Support Blocker and Support
    /// Enforcer"; with `full_sort` false only the support blockers/enforcers are placed.
    pub fn sort_volumes(&mut self, full_sort: bool) {
        if full_sort {
            self.volumes.sort_by(|vl, vr| vl.type_().cmp(&vr.type_()));
        } else {
            let key = |t: ModelVolumeType| if t > ModelVolumeType::ParameterModifier { t } else { ModelVolumeType::ParameterModifier };
            self.volumes.sort_by(|vl, vr| key(vl.type_()).cmp(&key(vr.type_())));
        }
    }

    /// Model.cpp:1603-1612 `add_instance()`
    pub fn add_instance(&mut self) -> &mut ModelInstance {
        let mut i = ModelInstance::new();
        i.set_model_object(Some(self.id()));
        self.instances.push(i);
        self.invalidate_bounding_box();
        self.instances.last_mut().unwrap()
    }

    /// Add a new instance at the specified position (loaders); returns its index.
    pub fn add_instance_at(&mut self, position: Point3F) -> usize {
        let id = self.instances.len();
        let mut i = ModelInstance::at(position);
        i.set_model_object(Some(self.id()));
        self.instances.push(i);
        id
    }

    /// Model.cpp:1614-1620 `add_instance(const ModelInstance &other)`
    pub fn add_instance_copy(&mut self, other: &ModelInstance) -> &mut ModelInstance {
        let mut i = ModelInstance::new_from(other);
        i.set_model_object(Some(self.id()));
        self.instances.push(i);
        self.invalidate_bounding_box();
        self.instances.last_mut().unwrap()
    }

    /// Model.cpp:1622-1630 `add_instance(offset, scaling_factor, rotation, mirror)`
    pub fn add_instance_with(&mut self, offset: &Vec3d, scaling_factor: &Vec3d, rotation: &Vec3d, mirror: &Vec3d) -> &mut ModelInstance {
        let instance = self.add_instance();
        instance.set_offset(offset);
        instance.set_scaling_factor(scaling_factor);
        instance.set_rotation(rotation);
        instance.set_mirror(mirror);
        instance
    }

    /// Model.cpp:1632-1638
    pub fn delete_instance(&mut self, idx: usize) {
        self.instances.remove(idx);
        self.invalidate_bounding_box();
    }

    /// Model.cpp:1640-1643
    pub fn delete_last_instance(&mut self) {
        self.delete_instance(self.instances.len() - 1);
    }

    /// Model.cpp:1645-1651
    pub fn clear_instances(&mut self) {
        self.instances.clear();
        self.invalidate_bounding_box();
    }

    /// Return the total number of instances for this object.
    pub fn instance_count(&self) -> usize {
        self.instances.len()
    }

    /// Set the printable flag for this object.
    pub fn set_printable(&mut self, printable: bool) {
        self.printable = printable;
    }

    /// Model.hpp:443
    pub fn invalidate_bounding_box(&mut self) {
        self.invalidate_bounding_box_cells();
    }

    /// Model.hpp:443, through the shared reference the cached members allow.
    fn invalidate_bounding_box_cells(&self) {
        self.m_bounding_box_valid.set(false);
        self.m_raw_bounding_box_valid.set(false);
        self.m_raw_mesh_bounding_box_valid.set(false);
    }

    /// Model.cpp:1655-1665 — the bounding box of the transformed instances, approximate and not
    /// snug, cached.
    pub fn bounding_box(&self) -> BoundingBoxf3 {
        if !self.m_bounding_box_valid.get() {
            self.m_bounding_box_valid.set(true);
            let raw_bbox = self.raw_mesh_bounding_box();
            let mut bb = BoundingBoxf3::new();
            for i in &self.instances {
                bbox_merge(&mut bb, &i.transform_bounding_box(&raw_bbox, false));
            }
            self.m_bounding_box.set(bb);
        }
        self.m_bounding_box.get()
    }

    /// Model.cpp:1667-1674
    pub fn bounding_box_in_assembly_view(&self) -> BoundingBoxf3 {
        let mut bb = BoundingBoxf3::new();
        let raw_bbox = self.raw_mesh_bounding_box();
        for i in &self.instances {
            bbox_merge(&mut bb, &i.transform_bounding_box_in_assembly_view(&raw_bbox, false));
        }
        self.m_bounding_box_in_assembly_view.set(bb);
        bb
    }

    /// Model.cpp:1677-1687 — a mesh containing all transformed instances of this object.
    pub fn mesh(&self) -> ModelMesh {
        let mut mesh = ModelMesh::new();
        let raw_mesh = self.raw_mesh();
        for i in &self.instances {
            let mut m = raw_mesh.clone();
            i.transform_mesh(&mut m, false);
            mesh.merge(&m);
        }
        mesh
    }

    /// Model.cpp:1692-1703 — the non-transformed sum of the model part volumes.
    pub fn raw_mesh(&self) -> ModelMesh {
        let mut mesh = ModelMesh::new();
        for v in &self.volumes {
            if v.is_model_part() {
                let mut vol_mesh = v.mesh().clone();
                vol_mesh.transform(&v.matrix(), false);
                mesh.merge(&vol_mesh);
            }
        }
        mesh
    }

    /// Model.cpp:1708-1735 — the same as raw_mesh() as a lightweight indexed_triangle_set.
    pub fn raw_indexed_triangle_set(&self) -> indexed_triangle_set {
        let mut num_vertices = 0;
        let mut num_faces = 0;
        for v in &self.volumes {
            if v.is_model_part() {
                num_vertices += v.mesh().its.vertices.len();
                num_faces += v.mesh().its.indices.len();
            }
        }
        let mut out = indexed_triangle_set::default();
        out.vertices.reserve(num_vertices);
        out.indices.reserve(num_faces);
        for v in &self.volumes {
            if v.is_model_part() {
                let mut i = out.vertices.len();
                let mut j = out.indices.len();
                out.vertices.extend_from_slice(&v.mesh().its.vertices);
                out.indices.extend_from_slice(&v.mesh().its.indices);
                let m = v.matrix();
                while i < out.vertices.len() {
                    let p = out.vertices[i];
                    let q = eigen_transform_point(&m, &Vec3d::new(p.x as f64, p.y as f64, p.z as f64));
                    out.vertices[i] = Vec3f::new(q.x as f32, q.y as f32, q.z as f32);
                    i += 1;
                }
                if v.is_left_handed() {
                    while j < out.indices.len() {
                        let tmp = out.indices[j][0];
                        out.indices[j][0] = out.indices[j][1];
                        out.indices[j][1] = tmp;
                        j += 1;
                    }
                }
            }
        }
        out
    }

    /// Model.cpp:1738-1748 — a snug bounding box of the non-transformed sum of the model part
    /// volumes, cached.
    pub fn raw_mesh_bounding_box(&self) -> BoundingBoxf3 {
        if !self.m_raw_mesh_bounding_box_valid.get() {
            self.m_raw_mesh_bounding_box_valid.set(true);
            let mut bb = BoundingBoxf3::new();
            for v in &self.volumes {
                if v.is_model_part() {
                    bbox_merge(&mut bb, &v.mesh().transformed_bounding_box(&v.matrix()));
                }
            }
            self.m_raw_mesh_bounding_box.set(bb);
        }
        self.m_raw_mesh_bounding_box.get()
    }

    /// Model.cpp:1750-1756 — the same over all volumes.
    pub fn full_raw_mesh_bounding_box(&self) -> BoundingBoxf3 {
        let mut bb = BoundingBoxf3::new();
        for v in &self.volumes {
            bbox_merge(&mut bb, &v.mesh().transformed_bounding_box(&v.matrix()));
        }
        bb
    }

    /// Model.cpp:1760-1774 — a transformed snug bounding box around the model part volumes,
    /// without the translation applied; errors out without instances.
    pub fn raw_bounding_box(&self) -> Result<BoundingBoxf3, &'static str> {
        if !self.m_raw_bounding_box_valid.get() {
            self.m_raw_bounding_box_valid.set(true);
            let mut bb = BoundingBoxf3::new();
            if self.instances.is_empty() {
                self.m_raw_bounding_box.set(bb);
                return Err("Can't call raw_bounding_box() with no instances");
            }

            let inst_matrix = self.instances[0].get_transformation().get_matrix(true, false, false, false);
            for v in &self.volumes {
                if v.is_model_part() {
                    bbox_merge(&mut bb, &v.mesh().transformed_bounding_box(&eigen_mul_affine(&inst_matrix, &v.matrix())));
                }
            }
            self.m_raw_bounding_box.set(bb);
        }
        Ok(self.m_raw_bounding_box.get())
    }

    /// Model.cpp:1777-1787 — an accurate snug bounding box of the transformed instance.
    pub fn instance_bounding_box(&self, instance_idx: usize, dont_translate: bool) -> BoundingBoxf3 {
        self.instance_bounding_box_of(&self.instances[instance_idx], dont_translate)
    }

    /// Model.cpp:1789-1797 `instance_bounding_box(const ModelInstance& instance, bool dont_translate)`
    pub fn instance_bounding_box_of(&self, instance: &ModelInstance, dont_translate: bool) -> BoundingBoxf3 {
        let mut bbox = BoundingBoxf3::new();
        let inst_mat = instance.get_transformation().get_matrix(dont_translate, false, false, false);
        for vol in &self.volumes {
            if vol.is_model_part() {
                bbox_merge(&mut bbox, &vol.mesh().transformed_bounding_box(&eigen_mul_affine(&inst_mat, &vol.matrix())));
            }
        }
        bbox
    }

    /// Model.cpp:1800-1803
    pub fn instance_convex_hull_bounding_box(&self, instance_idx: usize, dont_translate: bool) -> BoundingBoxf3 {
        self.instance_convex_hull_bounding_box_of(&self.instances[instance_idx], dont_translate)
    }

    /// Model.cpp:1805-1814 `instance_convex_hull_bounding_box(const ModelInstance* instance, bool dont_translate)`
    pub fn instance_convex_hull_bounding_box_of(&self, instance: &ModelInstance, dont_translate: bool) -> BoundingBoxf3 {
        let mut bb = BoundingBoxf3::new();
        let inst_matrix = instance.get_transformation().get_matrix(dont_translate, false, false, false);
        for v in &self.volumes {
            if v.is_model_part() {
                bbox_merge(&mut bb, &v.get_convex_hull().transformed_bounding_box(&eigen_mul_affine(&inst_matrix, &v.matrix())));
            }
        }
        bb
    }

    /// Model.cpp:1820-1868 — 2D convex hull of the projection of the transformed printable volumes
    /// into the XY plane (used by arrange).
    pub fn convex_hull_2d(&self, trafo_instance: &Transform3d) -> Polygon {
        let mut pts = Vec::new();
        for v in &self.volumes {
            if v.is_model_part() {
                let volume_hull = v.get_convex_hull_2d(trafo_instance);
                pts.extend_from_slice(&volume_hull.points);
            }
        }
        crate::geometry::convex_hull_points(pts)
    }

    /// Model.cpp:1870-1881
    pub fn center_around_origin(&mut self, include_modifiers: bool) {
        // calculate the displacements needed to
        // center this object around the origin
        let bb = if include_modifiers { self.full_raw_mesh_bounding_box() } else { self.raw_mesh_bounding_box() };

        // Shift is the vector from the center of the bounding box to the origin
        let shift = -from_bb(&bb.center());

        self.translate_vec(&shift);
        self.origin_translation += shift;
    }

    /// Model.cpp:1883-1905
    pub fn ensure_on_bed(&mut self, allow_negative_z: bool) {
        let mut z_offset = 0.0;

        if allow_negative_z {
            if self.parts_count() == 1 {
                let min_z = self.get_min_z();
                let max_z = self.get_max_z();
                if min_z >= SINKING_Z_THRESHOLD as f64 || max_z < 0.0 {
                    z_offset = -min_z;
                }
            } else {
                let max_z = self.get_max_z();
                if max_z < SINKING_MIN_Z_THRESHOLD {
                    z_offset = SINKING_MIN_Z_THRESHOLD - max_z;
                }
            }
        } else {
            z_offset = -self.get_min_z();
        }

        if z_offset != 0.0 {
            self.translate_instances(&(Vec3d::new(0.0, 0.0, 1.0) * z_offset));
        }
    }

    /// Model.cpp:1907-1912
    pub fn translate_instances(&mut self, vector: &Vec3d) {
        for i in 0..self.instances.len() {
            self.translate_instance(i, vector);
        }
    }

    /// Model.cpp:1914-1920
    pub fn translate_instance(&mut self, instance_idx: usize, vector: &Vec3d) {
        // assert(instance_idx < self.instances.len()): compiled out in the reference build (-DNDEBUG)
        let i = &mut self.instances[instance_idx];
        let offset = i.get_offset() + vector;
        i.set_offset(&offset);
        self.invalidate_bounding_box();
    }

    /// Model.hpp:478 `translate(const Vec3d &vector)`
    pub fn translate_vec(&mut self, vector: &Vec3d) {
        self.translate(vector[0], vector[1], vector[2]);
    }

    /// Model.cpp:1922-1930 `translate(double x, double y, double z)`
    pub fn translate(&mut self, x: f64, y: f64, z: f64) {
        for v in &mut self.volumes {
            v.translate(&Vec3d::new(x, y, z));
        }

        if self.m_bounding_box_valid.get() {
            let mut bb = self.m_bounding_box.get();
            bb.translate(x, y, z);
            self.m_bounding_box.set(bb);
        }
    }

    /// Model.cpp:1932-1938 `scale(const Vec3d &versor)`
    pub fn scale(&mut self, versor: &Vec3d) {
        for v in &mut self.volumes {
            v.scale(versor);
        }
        self.invalidate_bounding_box();
    }

    /// Model.cpp:3842-3853 — scale to fit by the scaling factor of the volumes (the size over all
    /// instances).
    pub fn scale_to_fit(&mut self, size: &Vec3d) {
        let orig_size = self.bounding_box().size();
        let factor = std_min(size.x / orig_size.x, std_min(size.y / orig_size.y, size.z / orig_size.z));
        self.scale(&Vec3d::new(factor, factor, factor));
    }

    /// Model.cpp:1940-1947 `rotate(double angle, Axis axis)`
    pub fn rotate_axis(&mut self, angle: f64, axis: Axis) {
        for v in &mut self.volumes {
            v.rotate_axis(angle, axis);
        }
        self.center_around_origin(true);
        self.invalidate_bounding_box();
    }

    /// Model.cpp:1949-1962 `rotate(double angle, const Vec3d& axis)`
    pub fn rotate(&mut self, angle: f64, axis: &Vec3d) {
        for v in &mut self.volumes {
            v.rotate(angle, axis);
        }

        //BBS update assemble transformation when modify volume rotation
        for i in &mut self.instances {
            i.rotate_assemble(-angle, axis);
        }

        self.center_around_origin(true);
        self.invalidate_bounding_box();
    }

    /// Model.cpp:1964-1970
    pub fn mirror(&mut self, axis: Axis) {
        for v in &mut self.volumes {
            v.mirror(axis);
        }
        self.invalidate_bounding_box();
    }

    /// Model.cpp:1973-1980 — only before the meshes of the volumes are shared.
    pub fn scale_mesh_after_creation(&mut self, scale: f32) {
        for v in &mut self.volumes {
            v.scale_geometry_after_creation(&Vec3f::new(scale, scale, scale));
            let offset = Vec3d::new(scale as f64, scale as f64, scale as f64).component_mul(&v.get_offset());
            v.set_offset(&offset);
        }
        self.invalidate_bounding_box();
    }

    /// Model.cpp:1982-2047 `convert_units(new_objects, conv_type, volume_idxs)`
    ///
    /// FIDELITY-NOTE: `vol->set_material(material_id, *volume->material())` needs the model's
    /// materials; the material id is kept and the caller registers the material.
    pub fn convert_units(&self, new_objects: &mut Vec<ModelObject>, conv_type: ConversionType, volume_idxs: &[i32]) {
        let mut new_object = ModelObject::new_clone(self);

        let koef: f32 = match conv_type {
            ConversionType::ConvFromInch => 25.4,
            ConversionType::ConvToInch => 0.0393700787,
            ConversionType::ConvFromMeter => 1000.0,
            ConversionType::ConvToMeter => 0.001,
        };

        new_object.set_model(None);
        new_object.brim_points.clear();
        new_object.clear_volumes();
        new_object.input_file.clear();

        let mut vol_idx = 0;
        for volume in &self.volumes {
            if !volume.mesh().empty() {
                let mesh = volume.mesh().clone();

                let new_objects_len = new_objects.len() as i32;
                let vol = new_object.add_volume_mesh(mesh, ModelVolumeType::ModelPart, true);
                vol.name = volume.name.clone();
                vol.set_type(volume.type_());
                // Don't copy the config's ID.
                vol.config.assign_config(&volume.config);
                vol.m_material_id = volume.material_id();
                vol.source.input_file = volume.source.input_file.clone();
                vol.source.object_idx = new_objects_len;
                vol.source.volume_idx = vol_idx;
                vol.source.is_converted_from_inches = volume.source.is_converted_from_inches;
                vol.source.is_converted_from_meters = volume.source.is_converted_from_meters;
                vol.source.is_from_builtin_objects = volume.source.is_from_builtin_objects;

                vol.supported_facets.assign(&volume.supported_facets);
                vol.fuzzy_skin_facets.assign(&volume.fuzzy_skin_facets);
                vol.seam_facets.assign(&volume.seam_facets);
                vol.mmu_segmentation_facets.assign(&volume.mmu_segmentation_facets);

                // Perform conversion only if the target "imperial" state is different from the current one.
                // This check supports conversion of "mixed" set of volumes, each with different "imperial" state.
                if volume_idxs.is_empty() || volume_idxs.contains(&vol_idx) {
                    vol.scale_geometry_after_creation(&Vec3f::new(koef, koef, koef));
                    let offset = Vec3d::new(koef as f64, koef as f64, koef as f64).component_mul(&volume.get_offset());
                    vol.set_offset(&offset);
                    if conv_type == ConversionType::ConvFromInch || conv_type == ConversionType::ConvToInch {
                        vol.source.is_converted_from_inches = conv_type == ConversionType::ConvFromInch;
                    }
                    if conv_type == ConversionType::ConvFromMeter || conv_type == ConversionType::ConvToMeter {
                        vol.source.is_converted_from_meters = conv_type == ConversionType::ConvFromMeter;
                    }
                    // assert(!vol.source.is_converted_from_inches || !vol.source.is_converted_from_meters): compiled out in the reference build (-DNDEBUG)
                } else {
                    vol.set_offset(&volume.get_offset());
                }
            }
            vol_idx += 1;
        }
        new_object.invalidate_bounding_box();

        new_objects.push(new_object);
    }

    /// Model.cpp:2049-2055
    pub fn materials_count(&self) -> usize {
        let material_ids: std::collections::BTreeSet<TModelMaterialId> = self.volumes.iter().map(|v| v.material_id()).collect();
        material_ids.len()
    }

    /// Model.cpp:2057-2064
    pub fn facets_count(&self) -> usize {
        let mut num = 0;
        for v in &self.volumes {
            if v.is_model_part() {
                num += v.mesh().facets_count();
            }
        }
        num
    }

    /// Model.cpp:2066-2073
    pub fn parts_count(&self) -> usize {
        let mut num = 0;
        for v in &self.volumes {
            if v.is_model_part() {
                num += 1;
            }
        }
        num
    }

    /// Model.hpp:499
    pub fn is_cut(&self) -> bool {
        self.cut_id.id().valid()
    }

    /// Model.cpp:2075-2082
    pub fn has_connectors(&self) -> bool {
        // assert(self.is_cut()): compiled out in the reference build (-DNDEBUG)
        for v in &self.volumes {
            if v.cut_info.is_connector {
                return true;
            }
        }
        false
    }

    /// Model.cpp:2084-2124
    pub fn get_connector_mesh(connector_attributes: CutConnectorAttributes, para: CutConnectorParas) -> indexed_triangle_set {
        use crate::triangle_mesh::{its_make_cone, its_make_cylinder, its_make_frustum_dowel, its_make_snap, its_make_thread};
        use std::f64::consts::PI;

        let sector_count = match connector_attributes.shape {
            CutConnectorShape::Triangle => 3,
            CutConnectorShape::Square => 4,
            CutConnectorShape::Circle => 360,
            CutConnectorShape::Hexagon => 6,
            _ => 1,
        };

        if connector_attributes.type_ == CutConnectorType::Snap {
            its_make_snap(1.0, 1.0, para.snap_space_proportion, para.snap_bulge_proportion)
        } else if connector_attributes.type_ == CutConnectorType::Thread {
            // --- OUR THREAD PREVIEW ---
            // Generates the standard 1x1x1 unit mesh and lets OpenGL handle the scaling
            its_make_thread(1.0, 1.0, 0.2f32 as f64, (2.0f32 * PI as f32 / sector_count as f32) as f64)
        } else if connector_attributes.style == CutConnectorStyle::Prizm {
            // --- ORIGINAL LOGIC BELOW ---
            its_make_cylinder(1.0, 1.0, 2.0 * PI / sector_count as f64)
        } else if connector_attributes.type_ == CutConnectorType::Plug {
            its_make_cone(1.0, 1.0, 2.0 * PI / sector_count as f64)
        } else {
            its_make_frustum_dowel(1.0, 1.0, sector_count)
        }
    }

    /// Model.cpp:2126-2154
    pub fn apply_cut_connectors(&mut self, name: &str) {
        if self.cut_connectors.is_empty() {
            return;
        }

        let mut connector_id = self.cut_id.connectors_cnt();
        let connectors = self.cut_connectors.clone();
        for connector in &connectors {
            let mesh = ModelMesh::from_its(Self::get_connector_mesh(connector.attribs, connector.paras));
            // Mesh will be centered when loading.
            let new_volume = self.add_volume_mesh(mesh, ModelVolumeType::NegativeVolume, true);

            let mut translate_transform = Transform3d::identity();
            translate_transform[(0, 3)] += connector.pos.x;
            translate_transform[(1, 3)] += connector.pos.y;
            translate_transform[(2, 3)] += connector.pos.z;
            let mut scale_transform = Transform3d::identity();
            scale_transform[(0, 0)] = connector.radius as f64;
            scale_transform[(1, 1)] = connector.radius as f64;
            scale_transform[(2, 2)] = connector.height as f64;

            // Transform the new modifier to be aligned inside the instance
            let t = eigen_mul_affine(&eigen_mul_affine(&translate_transform, &connector.rotation_m), &scale_transform);
            new_volume.set_transformation_matrix(&t);

            new_volume.cut_info = ModelVolumeCutInfo::new(
                connector.attribs.type_,
                connector.radius,
                connector.height,
                connector.radius_tolerance,
                connector.height_tolerance,
                false,
            );
            connector_id += 1;
            new_volume.name = format!("{}-{}", name, connector_id);
        }
        self.cut_id.increase_connectors_cnt(self.cut_connectors.len());

        // delete all connectors
        self.cut_connectors.clear();
    }

    /// Model.cpp:2156-2161 — invalidate the cut state of this object and its volumes.
    pub fn invalidate_cut(&mut self) {
        self.cut_id.invalidate();
        for volume in &mut self.volumes {
            volume.invalidate_cut_info();
        }
    }

    /// Model.cpp:2163-2169 — delete the volumes marked as connectors.
    pub fn delete_connectors(&mut self) {
        let mut id = self.volumes.len() as i32 - 1;
        while id >= 0 {
            if self.volumes[id as usize].is_cut_connector() {
                self.delete_volume(id as usize);
            }
            id -= 1;
        }
    }

    /// Model.cpp:2171-2178 `synchronize_model_after_cut()`: `model_objects` are the parent model's
    /// objects, `this_idx` this object's index there.
    pub fn synchronize_model_after_cut(model_objects: &mut [ModelObject], this_idx: usize) {
        let this_cut_id = model_objects[this_idx].cut_id.clone();
        for (i, obj) in model_objects.iter_mut().enumerate() {
            if i == this_idx || obj.cut_id.is_equal(&this_cut_id) {
                continue;
            }
            if obj.is_cut() && obj.cut_id.has_same_id(&this_cut_id) {
                obj.cut_id.copy(&this_cut_id);
            }
        }
    }

    /// Model.cpp:2180-2202
    pub fn apply_cut_attributes(&mut self, attributes: ModelObjectCutAttributes) {
        // we don't save cut information, if result will not contains all parts of initial object
        if !attributes.has(ModelObjectCutAttribute::KeepUpper)
            || !attributes.has(ModelObjectCutAttribute::KeepLower)
            || attributes.has(ModelObjectCutAttribute::InvalidateCutInfo)
        {
            return;
        }

        if self.cut_id.id().invalid() {
            self.cut_id.init();
        }

        {
            let mut cut_obj_cnt: i32 = -1;
            if attributes.has(ModelObjectCutAttribute::KeepUpper) {
                cut_obj_cnt += 1;
            }
            if attributes.has(ModelObjectCutAttribute::KeepLower) {
                cut_obj_cnt += 1;
            }
            if attributes.has(ModelObjectCutAttribute::CreateDowels) {
                cut_obj_cnt += 1;
            }
            if cut_obj_cnt > 0 {
                self.cut_id.increase_check_sum(cut_obj_cnt as usize);
            }
        }
    }

    /// Model.cpp:2204-2213 `clone_for_cut(ModelObject **obj)`
    pub fn clone_for_cut(&self) -> ModelObject {
        let mut obj = ModelObject::new_clone(self);
        obj.set_model(self.get_model());
        obj.clear_volumes();
        obj.input_file.clear();
        obj
    }

    /// Model.cpp:2238-2266
    pub fn calculate_cut_plane_inverse_matrix(&self, plane_points: &[Vec3d; 4]) -> Transform3d {
        let mut mid_point = Vec3d::zeros();
        for pt in plane_points {
            mid_point += pt;
        }
        mid_point /= plane_points.len() as f64;

        let mut movement = -mid_point;

        let v01 = plane_points[1] - plane_points[0];
        let v12 = plane_points[2] - plane_points[1];

        let mut plane_normal = Vec3d::new(
            v01.y.mul_add(v12.z, -(v01.z * v12.y)),
            v01.z.mul_add(v12.x, -(v01.x * v12.z)),
            v01.x.mul_add(v12.y, -(v01.y * v12.x)),
        );
        let n = eigen_squared_norm3(&plane_normal);
        if n > 0.0 {
            plane_normal /= n.sqrt();
        }

        let mut axis = Vec3d::zeros();
        let mut phi = 0.0;
        let mut matrix = Matrix3d::identity();
        crate::geometry::geometry::rotation_from_two_vectors(plane_normal, Vec3d::new(0.0, 0.0, 1.0), &mut axis, &mut phi, Some(&mut matrix));
        let angles = crate::geometry::geometry::extract_euler_angles_from_matrix(&matrix);

        movement = eigen_matrix3_mul_vec(&matrix, &movement);
        let mut transfo = Transform3d::identity();
        transfo[(0, 3)] += movement.x;
        transfo[(1, 3)] += movement.y;
        transfo[(2, 3)] += movement.z;
        let rot = eigen_matrix3_mul(
            &eigen_matrix3_mul(
                &eigen_angle_axis_matrix3(angles[2], &Vec3d::new(0.0, 0.0, 1.0)),
                &eigen_angle_axis_matrix3(angles[1], &Vec3d::new(0.0, 1.0, 0.0)),
            ),
            &eigen_angle_axis_matrix3(angles[0], &Vec3d::new(1.0, 0.0, 0.0)),
        );
        eigen_mul_affine(&transfo, &affine_of(&rot))
    }

    /// Model.cpp:2345-2371 `process_modifier_cut`: `upper` / `lower` the objects being built.
    pub fn process_modifier_cut(
        volume: &mut ModelVolume,
        instance_matrix: &Transform3d,
        inverse_cut_matrix: &Transform3d,
        attributes: ModelObjectCutAttributes,
        upper: Option<&mut ModelObject>,
        lower: Option<&mut ModelObject>,
    ) {
        let volume_matrix = eigen_mul_affine(instance_matrix, &volume.matrix());

        // Modifiers are not cut, but we still need to add the instance transformation
        // to the modifier volume transformation to preserve their shape properly.
        volume.set_transformation(&Transformation::from_transform(volume_matrix));

        if attributes.has(ModelObjectCutAttribute::CutToParts) {
            if let Some(u) = upper {
                u.add_volume_copy(volume, ModelVolumeType::Invalid);
            }
            return;
        }

        // Some logic for the negative volumes/connectors. Add only needed modifiers
        let bb = volume.mesh().transformed_bounding_box(&eigen_mul_affine(inverse_cut_matrix, &volume_matrix));
        let is_crossed_by_cut = bb.min.z <= 0.0 && bb.max.z >= 0.0;
        if attributes.has(ModelObjectCutAttribute::KeepUpper) && (bb.min.z >= 0.0 || is_crossed_by_cut) {
            if let Some(u) = upper {
                u.add_volume_copy(volume, ModelVolumeType::Invalid);
            }
        }
        if attributes.has(ModelObjectCutAttribute::KeepLower) && (bb.max.z <= 0.0 || is_crossed_by_cut) {
            if let Some(l) = lower {
                l.add_volume_copy(volume, ModelVolumeType::Invalid);
            }
        }
    }

    /// Model.cpp:2720-2837 `split(ModelObjectPtrs* new_objects)`: the new objects are made as
    /// `m_model->add_object()` makes them and returned in `new_objects`; the caller adds them to the
    /// model (FIDELITY-NOTE: the parent pointer).
    ///
    /// FIDELITY-NOTE: `FaceDetector::detect_exterior_face()` marks face properties of the meshes,
    /// which the crate's `indexed_triangle_set` has no member for; it is not run.
    pub fn split(&self, new_objects: &mut Vec<ModelObject>) {
        let mut all_meshes: Vec<ModelMesh> = Vec::with_capacity(self.volumes.len() * 5);
        let mut all_transfos: Vec<Transform3d> = Vec::new();
        let mut volume_mesh_counts: Vec<(usize, usize)> = Vec::new();
        let is_multi_volume_object = self.volumes.len() > 1;

        for (volume_idx, volume) in self.volumes.iter().enumerate() {
            if volume.type_() != ModelVolumeType::ModelPart {
                continue;
            }

            // splited volume should not be text object
            if !is_multi_volume_object {
                //BBS: not multi volume object, then split mesh.
                let volume_meshes = volume.mesh().split(1.0);
                let mut mesh_count = 0;
                for mesh in volume_meshes {
                    if mesh.facets_count() < 3 {
                        continue;
                    }

                    all_meshes.push(mesh);
                    all_transfos.push(volume.matrix());
                    mesh_count += 1;
                }
                volume_mesh_counts.push((volume_idx, mesh_count));
            } else {
                //BBS: multi volume object, then only split to volume
                if volume.mesh().facets_count() >= 3 {
                    all_meshes.push(volume.mesh().clone());
                    all_transfos.push(volume.matrix());
                    volume_mesh_counts.push((volume_idx, 1));
                }
            }
        }

        let mut all_meshes: Vec<Option<ModelMesh>> = all_meshes.into_iter().map(Some).collect();
        let mut volume_mesh_begin = 0;
        for &(vidx, count) in &volume_mesh_counts {
            let volume = &self.volumes[vidx];

            let mut meshes: Vec<ModelMesh> = Vec::new();
            for mesh_idx in volume_mesh_begin..volume_mesh_begin + count {
                meshes.push(all_meshes[mesh_idx].take().unwrap());
            }
            volume_mesh_begin += count;

            let mut counter = 1;
            let meshes_len = meshes.len();
            for mesh in meshes {
                // FIXME: crashes if not satisfied
                if mesh.facets_count() < 3 {
                    continue;
                }

                // XXX: this seems to be the only real usage of m_model, maybe refactor this so that it's not needed?
                // m_model->add_object(): the new object's parent is this object's model.
                let mut new_object = ModelObject::new_for_model();
                new_object.set_model(self.m_model);
                //BBS: refine the config logic
                //use object as basic, and add volume's config
                if meshes_len == 1 {
                    new_object.name = volume.name.clone();
                } else {
                    new_object.name = if meshes_len > 1 {
                        let s = format!("{}_{}", self.name, counter);
                        counter += 1;
                        s
                    } else {
                        self.name.clone()
                    };
                }
                new_object.config.assign_config(&self.config);
                new_object.config.apply(&volume.config, true);

                new_object.instances.reserve(self.instances.len());
                for model_instance in &self.instances {
                    new_object.add_instance_copy(model_instance);
                }
                let new_vol_idx = {
                    new_object.add_volume(volume, mesh);
                    new_object.volumes.len() - 1
                };
                {
                    let new_vol = &mut new_object.volumes[new_vol_idx];

                    if is_multi_volume_object {
                        // BBS: volume geometry not changed, so we can keep the color paint facets
                        if new_vol.mmu_segmentation_facets.timestamp() == volume.mmu_segmentation_facets.timestamp() {
                            new_vol.mmu_segmentation_facets.reset(); // BBS: let next assign take effect
                        }
                        new_vol.mmu_segmentation_facets.assign(&volume.mmu_segmentation_facets);

                        // Carry over the assembly GUIDs (add_volume's new-mesh ctor does not copy them).
                        new_vol.set_part_guid(&volume.part_guid());
                        new_vol.set_assembly_src_guid(volume.assembly_src_guid());
                    }

                    // BBS: clear volume's config, as we already set them into object
                    new_vol.config.reset();
                }

                let new_vol_offset = new_object.volumes[new_vol_idx].get_offset();
                for model_instance in &mut new_object.instances {
                    let shift = eigen_transform_point(&model_instance.get_transformation().get_matrix(true, false, false, false), &new_vol_offset);
                    let offset = model_instance.get_offset() + shift;
                    model_instance.set_offset(&offset);

                    // BBS: the copied instance keeps the source assemble matrix but resets the initialized
                    let assemble_trafo = model_instance.get_assemble_transformation();
                    model_instance.set_assemble_transformation(&assemble_trafo);
                }

                let new_vol = &mut new_object.volumes[new_vol_idx];
                // BBS: keep the assembly-view world transform identical across split.
                new_vol.set_assemble_transformation(volume.get_assemble_transformation());

                new_vol.set_offset(&Vec3d::zeros());
                // reset the source to disable reload from disk
                new_vol.source = ModelVolumeSource::default();
                new_objects.push(new_object);
            }
        }
        let _ = all_transfos;
    }

    /// Model.cpp:2840-2858
    pub fn merge(&mut self) {
        if self.volumes.len() == 1 {
            // We can't merge meshes if there's just one volume
            return;
        }

        let mut mesh = ModelMesh::new();

        for volume in &self.volumes {
            if !volume.mesh().empty() {
                mesh.merge(volume.mesh());
            }
        }

        self.clear_volumes();
        self.add_volume_mesh(mesh, ModelVolumeType::ModelPart, true);
    }

    /// Model.cpp:2860-2937 `merge_volumes(std::vector<int>& vol_indeces)`: a clone of this object
    /// whose volumes `vol_indeces` are merged into one, the paint re-projected onto it; the source
    /// meshes of this object are released.
    pub fn merge_volumes(&mut self, vol_indeces: &[i32]) -> Vec<ModelObject> {
        let mut res = Vec::new();
        if self.volumes.len() == 1 {
            // We can't merge meshes if there's just one volume
            return res;
        }

        let mut upper = ModelObject::new_clone(self);
        upper.brim_points.clear();
        upper.clear_volumes();
        upper.input_file.clear();

        let mut mesh = ModelMesh::new();
        // Preserve painting across the merge. Each part is collected in object frame with
        // the filament it prints in, so a part that is unpainted but assigned a different
        // filament keeps its color once the parts share a single volume.
        let mut painted: Vec<usize> = Vec::new();
        for &i in vol_indeces {
            let volume = &self.volumes[i as usize];
            if !volume.mesh().empty() {
                painted.push(i as usize);

                let mut mesh_ = volume.mesh().clone();
                mesh_.transform(&volume.matrix(), true);
                mesh.merge(&mesh_);
            }
        }

        let merged_idx = upper.volumes.len();
        upper.add_volume_mesh(mesh, ModelVolumeType::ModelPart, true);
        // Reproject the source paint (brush strokes + baked solid part color) onto the
        // merged mesh, then free the source meshes.
        if !painted.is_empty() {
            let paint_sources: Vec<crate::paint_reproject::PaintSourceVolume> = painted
                .iter()
                .map(|&i| {
                    let volume = &self.volumes[i];
                    crate::paint_reproject::PaintSourceVolume {
                        mesh: Some(&volume.mesh().its),
                        supported: Some(volume.supported_facets.annotation()),
                        seam: Some(volume.seam_facets.annotation()),
                        mmu: Some(volume.mmu_segmentation_facets.annotation()),
                        fuzzy: Some(volume.fuzzy_skin_facets.annotation()),
                        source_to_destination: volume.matrix(),
                        base_filament: volume.extruder_id(self), // bake solid part color into paint
                    }
                })
                .collect();
            let vol = &mut upper.volumes[merged_idx];
            let (mut sup, mut seam, mut mmu, mut fuzzy) = (
                vol.supported_facets.annotation().clone(),
                vol.seam_facets.annotation().clone(),
                vol.mmu_segmentation_facets.annotation().clone(),
                vol.fuzzy_skin_facets.annotation().clone(),
            );
            let _ = crate::paint_reproject::reproject_paint_from_volumes(
                &vol.mesh().its,
                &paint_sources,
                &mut sup,
                &mut seam,
                &mut mmu,
                &mut fuzzy,
                None,
                None,
                None,
                /*concatenated_result=*/ true,
            );
            vol.supported_facets.set_data(sup);
            vol.seam_facets.set_data(seam);
            vol.mmu_segmentation_facets.set_data(mmu);
            vol.fuzzy_skin_facets.set_data(fuzzy);
        }
        for &i in vol_indeces {
            if !self.volumes[i as usize].mesh().empty() {
                self.volumes[i as usize].reset_mesh();
            }
        }
        for i in 0..self.volumes.len() {
            if vol_indeces.contains(&(i as i32)) {
                let vol = &mut upper.volumes[merged_idx];
                vol.name = "Merged Parts".to_string();
                vol.config.assign_config(&self.volumes[i].config);
            } else {
                upper.add_volume_copy(&self.volumes[i], ModelVolumeType::Invalid);
            }
        }
        upper.invalidate_bounding_box();
        res.push(upper);
        res
    }

    /// Model.cpp:2943-2993 — bake the instance rotation (and non-uniform scaling and mirroring) of
    /// `instance_idx` into the meshes, when it is not a multiple of ninety degrees.
    pub fn bake_xy_rotation_into_meshes(&mut self, instance_idx: usize) {
        // assert(instance_idx < self.instances.len()): compiled out in the reference build (-DNDEBUG)

        let reference_trafo = *self.instances[instance_idx].get_transformation();

        if crate::geometry::geometry::is_rotation_ninety_degrees(&reference_trafo.get_rotation()) {
            // nothing to do, scaling in the world coordinate space is possible in the representation of Geometry::Transformation.
            return;
        }

        let left_handed = reference_trafo.is_left_handed();
        let has_mirrorring = !eigen_is_approx_vec3(&reference_trafo.get_mirror(), &Vec3d::new(1.0, 1.0, 1.0));
        let sf = reference_trafo.get_scaling_factor();
        let uniform_scaling = (sf.x - sf.y).abs() < EPSILON && (sf.x - sf.z).abs() < EPSILON;
        let new_scaling_factor = if uniform_scaling { sf.x } else { 1.0 };

        // Adjust the instances.
        for model_instance in &mut self.instances {
            let rot = Vec3d::new(0.0, 0.0, crate::geometry::geometry::rotation_diff_z(&reference_trafo.get_rotation(), &model_instance.get_rotation()));
            model_instance.set_rotation(&rot);
            model_instance.set_scaling_factor(&Vec3d::new(new_scaling_factor, new_scaling_factor, new_scaling_factor));
            model_instance.set_mirror(&Vec3d::new(1.0, 1.0, 1.0));
        }

        // Adjust the meshes.
        // Transformation to be applied to the meshes.
        let mesh_trafo_3x3 = linear_of(&reference_trafo.get_matrix(true, false, uniform_scaling, !has_mirrorring));
        let volume_offset_correction =
            eigen_mul_affine(&eigen_inverse_affine(&self.instances[instance_idx].get_transformation().matrix()), &reference_trafo.matrix());
        for model_volume in &mut self.volumes {
            let volume_trafo = *model_volume.get_transformation();
            let volume_left_handed = volume_trafo.is_left_handed();
            let volume_has_mirrorring = !eigen_is_approx_vec3(&volume_trafo.get_mirror(), &Vec3d::new(1.0, 1.0, 1.0));
            let vsf = volume_trafo.get_scaling_factor();
            let volume_uniform_scaling = (vsf.x - vsf.y).abs() < EPSILON && (vsf.x - vsf.z).abs() < EPSILON;
            let volume_new_scaling_factor = if volume_uniform_scaling { vsf.x } else { 1.0 };
            // Transform the mesh.
            let volume_trafo_3x3 = linear_of(&volume_trafo.get_matrix(true, false, volume_uniform_scaling, !volume_has_mirrorring));
            // Following method creates a new shared_ptr<TriangleMesh>
            model_volume.transform_this_mesh_matrix3(&eigen_matrix3_mul(&mesh_trafo_3x3, &volume_trafo_3x3), left_handed != volume_left_handed);
            // Reset the rotation, scaling and mirroring.
            model_volume.set_rotation(&Vec3d::zeros());
            model_volume.set_scaling_factor(&Vec3d::new(volume_new_scaling_factor, volume_new_scaling_factor, volume_new_scaling_factor));
            model_volume.set_mirror(&Vec3d::new(1.0, 1.0, 1.0));
            // Move the reference point of the volume to compensate for the change of the instance trafo.
            model_volume.set_offset(&eigen_transform_point(&volume_offset_correction, &volume_trafo.get_offset()));
            // reset the source to disable reload from disk
            model_volume.source = ModelVolumeSource::default();
        }

        self.invalidate_bounding_box();
    }

    /// Model.cpp:2995-3006
    pub fn get_min_z(&self) -> f64 {
        if self.instances.is_empty() {
            0.0
        } else {
            let mut min_z = f64::MAX;
            for i in 0..self.instances.len() {
                min_z = std_min(min_z, self.get_instance_min_z(i));
            }
            min_z
        }
    }

    /// Model.cpp:3008-3019
    pub fn get_max_z(&self) -> f64 {
        if self.instances.is_empty() {
            0.0
        } else {
            let mut max_z = -f64::MAX;
            for i in 0..self.instances.len() {
                max_z = std_max(max_z, self.get_instance_max_z(i));
            }
            max_z
        }
    }

    /// Model.cpp:3021-3055
    pub fn get_instance_min_z(&self, instance_idx: usize) -> f64 {
        let mut min_z = f64::MAX;

        let inst = &self.instances[instance_idx];
        let mi = inst.get_matrix(true, false, false, false);

        for v in &self.volumes {
            if !v.is_model_part() {
                continue;
            }

            let mv = eigen_mul_affine(&mi, &v.matrix());
            let hull = v.get_convex_hull();
            //BBS: in some case the convex hull is empty due to the qhull algo
            //use the original mesh instead
            let src = if hull.its.indices.is_empty() { v.mesh() } else { hull };
            for facet in &src.its.indices {
                for i in 0..3 {
                    let p = src.its.vertices[facet[i] as usize];
                    min_z = std_min(min_z, eigen_transform_point(&mv, &Vec3d::new(p.x as f64, p.y as f64, p.z as f64)).z);
                }
            }
        }

        //BBS: add some logic to avoid wrong compute for min_z
        if min_z == f64::MAX {
            min_z = 0.0;
        }
        min_z + inst.get_offset_axis(Axis::Z)
    }

    /// Model.cpp:3057-3076
    pub fn get_instance_max_z(&self, instance_idx: usize) -> f64 {
        let mut max_z = -f64::MAX;

        let inst = &self.instances[instance_idx];
        let mi = inst.get_matrix(true, false, false, false);

        for v in &self.volumes {
            if !v.is_model_part() {
                continue;
            }

            let mv = eigen_mul_affine(&mi, &v.matrix());
            let hull = v.get_convex_hull();
            for facet in &hull.its.indices {
                for i in 0..3 {
                    let p = hull.its.vertices[facet[i] as usize];
                    max_z = std_max(max_z, eigen_transform_point(&mv, &Vec3d::new(p.x as f64, p.y as f64, p.z as f64)).z);
                }
            }
        }

        max_z + inst.get_offset_axis(Axis::Z)
    }

    /// Model.cpp:3078-3146 `update_instances_print_volume_state(const BuildVolume &build_volume)`:
    /// the number of instances inside.
    pub fn update_instances_print_volume_state(&mut self, build_volume: &crate::build_volume::BuildVolume) -> u32 {
        use crate::build_volume::{ObjectState, Type};
        let mut num_printable = 0;
        const INSIDE: u32 = 1;
        const OUTSIDE: u32 = 2;

        let volumes = &self.volumes;
        for model_instance in &mut self.instances {
            let mut inside_outside = 0u32;
            for vol in volumes {
                if vol.is_model_part() {
                    //BBS: add bounding box empty check logic, for some volume is empty before split(it will be removed after split to object)
                    let bb = vol.get_convex_hull().bounding_box();
                    let size = bb.size();
                    if size.x == 0.0 || size.y == 0.0 || size.z == 0.0 {
                        continue;
                    }

                    let matrix = eigen_mul_affine(&model_instance.matrix(), &vol.matrix());
                    let state = match build_volume.volume_type() {
                        Type::Rectangle => {
                            let transformed_bb = vol.get_convex_hull().transformed_bounding_box(&matrix);
                            build_volume.volume_state_bbox(&transformed_bb, true)
                        }
                        _ => {
                            // FIDELITY-NOTE: `object_state(its, matrix.cast<float>(), true)` takes the
                            // crate's `Transform3D` of the double matrix.
                            let mut t = Transform3D::identity();
                            for r in 0..3 {
                                for c in 0..4 {
                                    t.matrix[c * 4 + r] = matrix[(r, c)] as f32 as f64;
                                }
                            }
                            // (the build volume's own `indexed_triangle_set` type, the same vertices and faces)
                            let its = crate::triangle_set_sampling::indexed_triangle_set {
                                vertices: vol.mesh().its.vertices.clone(),
                                indices: vol.mesh().its.indices.iter().map(|f| crate::triangle_set_sampling::Vec3i::new(f[0], f[1], f[2])).collect(),
                            };
                            build_volume.object_state(&its, &t, true, true)
                        }
                    };

                    if state == ObjectState::Inside {
                        // Volume is completely inside.
                        inside_outside |= INSIDE;
                    } else if state == ObjectState::Outside {
                        // Volume is completely outside.
                        inside_outside |= OUTSIDE;
                    } else if state == ObjectState::Below {
                        // Volume below the print bed, thus it is completely outside, however this does not prevent the object to be printable
                        // if some of its volumes are still inside the build volume.
                    } else {
                        // Volume colliding with the build volume.
                        inside_outside |= INSIDE | OUTSIDE;
                    }
                }
            }
            model_instance.print_volume_state = if inside_outside == (INSIDE | OUTSIDE) {
                ModelInstanceEPrintVolumeState::ModelInstancePvsPartlyOutside
            } else if inside_outside == INSIDE {
                ModelInstanceEPrintVolumeState::ModelInstancePvsInside
            } else {
                ModelInstanceEPrintVolumeState::ModelInstancePvsFullyOutside
            };
            if inside_outside == INSIDE {
                num_printable += 1;
            }
        }
        num_printable
    }

    /// Model.hpp:402-407 `get_config_value<T>(global_config, config_option)`: this object's option,
    /// else the global one.
    pub fn get_config_value<'a>(&'a self, global_config: &'a DynamicPrintConfig, config_option: &str) -> Option<&'a dyn crate::config::ConfigOption> {
        use crate::config::ConfigOptionResolver;
        if self.config.has(config_option) {
            self.config.option(config_option)
        } else {
            global_config.optptr(config_option)
        }
    }

    /// Model.cpp:3148-3191 — print object statistics to the console.
    pub fn print_info(&self) {
        println!(
            "[{}]",
            std::path::Path::new(&self.input_file).file_name().map(|s| s.to_string_lossy().into_owned()).unwrap_or_default()
        );

        let mut mesh = self.raw_mesh();
        let bb = mesh.bounding_box();
        let size = bb.size();
        println!("size_x = {:.6}", size.x);
        println!("size_y = {:.6}", size.y);
        println!("size_z = {:.6}", size.z);
        println!("min_x = {:.6}", bb.min.x);
        println!("min_y = {:.6}", bb.min.y);
        println!("min_z = {:.6}", bb.min.z);
        println!("max_x = {:.6}", bb.max.x);
        println!("max_y = {:.6}", bb.max.y);
        println!("max_z = {:.6}", bb.max.z);
        println!("number_of_facets = {}", mesh.facets_count());

        println!("manifold = {}", if mesh.stats().manifold() { "yes" } else { "no" });
        if mesh.stats().open_edges > 0 {
            println!("open_edges = {}", mesh.stats().open_edges);
        }

        if mesh.stats().repaired() {
            let stats = mesh.stats().repaired_errors;
            if stats.degenerate_facets > 0 {
                println!("degenerate_facets = {}", stats.degenerate_facets);
            }
            if stats.edges_fixed > 0 {
                println!("edges_fixed = {}", stats.edges_fixed);
            }
            if stats.facets_removed > 0 {
                println!("facets_removed = {}", stats.facets_removed);
            }
            if stats.facets_reversed > 0 {
                println!("facets_reversed = {}", stats.facets_reversed);
            }
            if stats.backwards_edges > 0 {
                println!("backwards_edges = {}", stats.backwards_edges);
            }
        }
        println!("number_of_parts =  {}", mesh.stats().number_of_parts);
        println!("volume = {:.6}", mesh.volume());
    }

    /// Model.cpp:3193-3211
    pub fn get_export_filename(&self) -> String {
        let mut ret = self.input_file.clone();

        if !self.name.is_empty() {
            if ret.is_empty() {
                // input_file was empty, just use name
                ret = self.name.clone();
            } else {
                // Replace file name in input_file with name, but keep the path and file extension.
                let name_parent_empty = std::path::Path::new(&self.name).parent().map(|p| p.as_os_str().is_empty()).unwrap_or(true);
                ret = if name_parent_empty {
                    let parent = std::path::Path::new(&ret).parent().map(|p| p.to_path_buf()).unwrap_or_default();
                    parent.join(&self.name).to_string_lossy().into_owned()
                } else {
                    self.name.clone()
                };
            }
        }

        ret
    }

    /// Model.cpp:3213-3239 — full stl statistics of all the object's meshes.
    pub fn get_object_stl_stats(&self) -> TriangleMeshStats {
        let mut full_stats = TriangleMeshStats::default();
        full_stats.volume = 0.0;

        // fill full_stats from all objet's meshes
        for volume in &self.volumes {
            let stats = volume.mesh().stats();

            // initialize full_stats (for repaired errors)
            full_stats.open_edges += stats.open_edges;
            full_stats.repaired_errors.merge(&stats.repaired_errors);

            // another used satistics value
            if volume.is_model_part() {
                let trans = if self.instances.is_empty() {
                    volume.matrix()
                } else {
                    eigen_mul_affine(&volume.matrix(), &self.instances[0].matrix())
                };
                full_stats.volume = (full_stats.volume as f64 + stats.volume as f64 * eigen_determinant3(&linear_of(&trans)).abs()) as f32;
                full_stats.number_of_parts += stats.number_of_parts;
            }
        }

        full_stats
    }

    /// Model.cpp:3241-3250
    pub fn get_repaired_errors_count(&self, vol_idx: i32) -> i32 {
        if vol_idx >= 0 {
            return self.volumes[vol_idx as usize].get_repaired_errors_count();
        }

        let stats = self.get_object_stl_stats().repaired_errors;

        stats.degenerate_facets + stats.edges_fixed + stats.facets_removed + stats.facets_reversed + stats.backwards_edges
    }
}

// =============================================================================================
// Model.cpp:2434-2506 — cut helpers on objects
// =============================================================================================

/// Model.cpp:2434-2445 `invalidate_translations(ModelObject* object, const ModelInstance* src_instance)`
pub fn invalidate_translations(object: &mut ModelObject, src_instance_idx: usize) {
    let src_offset = object.instances[src_instance_idx].get_offset();
    if !eigen_is_approx_vec3(&object.origin_translation, &Vec3d::zeros()) && eigen_is_approx_vec3(&src_offset, &Vec3d::zeros()) {
        object.center_around_origin(true);
        let t = -object.origin_translation;
        object.translate_instances(&t);
        object.origin_translation = Vec3d::zeros();
    } else {
        object.invalidate_bounding_box();
        object.center_around_origin(true);
    }
}

/// Model.cpp:920-926 `auto_extruder_id(max_extruders, cntr)`: the next extruder ID in
/// (1, max_extruders).
pub fn auto_extruder_id(max_extruders: u32, cntr: &mut u32) -> i32 {
    *cntr += 1;
    let out = *cntr as i32;
    if *cntr == max_extruders {
        *cntr = 0;
    }
    out
}

/// Model.cpp:284-301 `add_textured_mesh_to_model(model, tex_mesh, input_file)`
pub fn add_textured_mesh_to_model(model: &mut Model, tex_mesh: &crate::texture_painting::TexturedMesh, input_file: &str) {
    let object_name = std::path::Path::new(input_file).file_name().map(|s| s.to_string_lossy().into_owned()).unwrap_or_default();

    let mut its = indexed_triangle_set::default();
    its.vertices = tex_mesh.vertices.iter().map(|v| Vec3f::new(v[0], v[1], v[2])).collect();
    its.indices = tex_mesh.indices.iter().map(|i| Vec3i::new(i[0], i[1], i[2])).collect();

    crate::triangle_mesh::its_merge_vertices(&mut its, true);
    crate::triangle_mesh::its_remove_degenerate_faces(&mut its, true);
    crate::triangle_mesh::its_compactify_vertices(&mut its, true);

    model.add_object_with_mesh(&object_name, input_file, ModelMesh::from_its(its));
}

/// Model.cpp:2447-2506 `reset_instance_transformation(object, src_instance_idx, cut_matrix,
/// place_on_cut, flip, local_displace)`: reset the instance transformations except the offset
/// and the Z rotation, and update the assemble matrices.
pub fn reset_instance_transformation(
    object: &mut ModelObject,
    src_instance_idx: usize,
    cut_matrix: &Transform3d,
    place_on_cut: bool,
    flip: bool,
    local_displace: &Vec3d,
) {
    use crate::geometry::geometry::rotation_transform;
    use std::f64::consts::PI;

    // Reset instance transformation except offset and Z-rotation

    for i in 0..object.instances.len() {
        let mut instance_transformation_copy = *object.instances[i].get_transformation();
        instance_transformation_copy.set_offset(&Vec3d::zeros());
        if object.volumes.len() == 1 {
            instance_transformation_copy.set_offset(&(-object.volumes[0].get_offset()));
        }

        if i == src_instance_idx && object.volumes.len() == 1 {
            invalidate_translations(object, i);
        }

        let obj_instance = &mut object.instances[i];
        let offset = obj_instance.get_offset();
        let rot_z = obj_instance.get_rotation().z;

        obj_instance.set_transformation(&Transformation::new());

        let displace = if eigen_is_approx_vec3(local_displace, &Vec3d::zeros()) {
            Vec3d::zeros()
        } else {
            eigen_transform_point(&rotation_transform(&obj_instance.get_rotation()), local_displace)
        };
        obj_instance.set_offset(&(offset + displace));

        let mut rotation = Vec3d::zeros();
        if !flip && !place_on_cut {
            if i != src_instance_idx {
                rotation[2] = rot_z;
            }
        } else {
            let mut rotation_matrix = Transform3d::identity();
            if flip {
                rotation_matrix = rotation_transform(&(Vec3d::new(1.0, 0.0, 0.0) * PI));
            }

            if place_on_cut {
                rotation_matrix = eigen_mul_affine(
                    &rotation_matrix,
                    &eigen_inverse_affine(&Transformation::from_transform(*cut_matrix).get_matrix(true, false, true, true)),
                );
            }

            if i != src_instance_idx {
                rotation_matrix = eigen_mul_affine(&rotation_transform(&(Vec3d::new(0.0, 0.0, 1.0) * rot_z)), &rotation_matrix);
            }

            rotation = Transformation::from_transform(rotation_matrix).get_rotation();
        }

        obj_instance.set_rotation(&rotation);

        // update the assemble matrix
        let assemble_matrix = obj_instance.get_assemble_transformation().matrix();
        let instance_inverse_matrix = eigen_inverse_affine(&instance_transformation_copy.matrix());
        let mut new_instance_inverse_matrix =
            eigen_mul_affine(&instance_inverse_matrix, &eigen_inverse_affine(&obj_instance.get_transformation().get_matrix(true, false, false, false)));
        if place_on_cut {
            // reset the rotation of cut plane
            new_instance_inverse_matrix = eigen_mul_affine(
                &new_instance_inverse_matrix,
                &eigen_inverse_affine(&Transformation::from_transform(*cut_matrix).get_matrix(true, false, true, true)),
            );
        }
        let new_assemble_transform = eigen_mul_affine(&assemble_matrix, &new_instance_inverse_matrix);
        obj_instance.set_assemble_from_transform(&new_assemble_transform);
    }
}

/// Model.cpp:1295-1311 `add_cut_volume(mesh, object, src_volume, cut_matrix, suffix, type)`:
/// `materials` is the parent model's material map.
pub fn add_cut_volume(
    mesh: &mut ModelMesh,
    object: &mut ModelObject,
    src_volume: &ModelVolume,
    cut_matrix: &Transform3d,
    suffix: &str,
    type_: ModelVolumeType,
    materials: &mut ModelMaterialMap,
) {
    if mesh.empty() {
        return;
    }

    mesh.transform(cut_matrix, false);
    let vol = object.add_volume_mesh(mesh.clone(), ModelVolumeType::ModelPart, true);
    vol.set_type(type_);

    vol.name = format!("{}{}", src_volume.name, suffix);
    // Don't copy the config's ID.
    vol.config.assign_config(&src_volume.config);
    if let Some(material) = materials.get(&src_volume.material_id()).cloned() {
        vol.set_material(src_volume.material_id(), &material, materials);
    } else {
        vol.m_material_id = src_volume.material_id();
    }
    vol.cut_info = src_volume.cut_info;
}

// =============================================================================================
// PrintConfig.cpp:11450-11506 — the bed shape of a configuration (used by setPrintSpeedTable)
// =============================================================================================

/// PrintConfig.cpp:11450-11456 `static Points to_points(const std::vector<Vec2d> &dpts)`
fn print_config_to_points(dpts: &[crate::geometry::Vec2d]) -> Vec<crate::Point> {
    let mut pts = Vec::with_capacity(dpts.len());
    for v in dpts {
        pts.push(crate::Point::new(coord_scale(v.x), coord_scale(v.y)));
    }
    pts
}

/// PrintConfig.cpp:11458-11475 `Polygon get_shared_poly(const std::vector<Pointfs>& extruder_polys)`
///
/// FIDELITY-NOTE: a function of PrintConfig.cpp, which `print_config.rs` has not ported yet; it is
/// here for `Model::set_print_speed_table` and belongs there.
fn get_shared_poly(extruder_polys: &[Vec<crate::geometry::Vec2d>]) -> Polygon {
    let mut result = Polygon::new();
    for index in 0..extruder_polys.len() {
        let extruder_area = &extruder_polys[index];
        if index == 0 {
            result.points = print_config_to_points(extruder_area);
        } else {
            let mut extruer_poly = Polygon::new();
            extruer_poly.points = print_config_to_points(extruder_area);
            let result_polygon = crate::clipper_utils::intersection_polygons_raw_clib(&[extruer_poly], &[result.clone()]);
            if result_polygon.is_empty() {
                return Polygon::new();
            }
            result = result_polygon[0].clone();
        }
    }
    result
}

/// PrintConfig.cpp:11477-11506 `Points get_bed_shape(const DynamicPrintConfig &config, bool use_share)`
///
/// FIDELITY-NOTE: a function of PrintConfig.cpp, which `print_config.rs` has not ported yet; it is
/// here for `Model::set_print_speed_table` and belongs there.
fn get_bed_shape(config: &DynamicPrintConfig, use_share: bool) -> Vec<crate::Point> {
    let bed_shape_opt = config.opt::<crate::config::ConfigOptionPoints>("printable_area");
    let Some(bed_shape_opt) = bed_shape_opt else {
        // Here, it is certain that the bed shape is missing, so an infinite one
        // has to be used, but still, the center of bed can be queried
        if let Some(center_opt) = config.opt::<crate::config::ConfigOptionPoint>("center") {
            return vec![crate::Point::new(
                (center_opt.value.x / SCALING_FACTOR) as crate::Coord,
                (center_opt.value.y / SCALING_FACTOR) as crate::Coord,
            )];
        }

        return Vec::new();
    };

    let mut bed_poly = Polygon::new();
    if use_share {
        let extruder_area_opt = config.opt::<crate::config::ConfigOptionPointsGroups>("extruder_printable_area");
        if let Some(extruder_area_opt) = extruder_area_opt.filter(|o| !o.values.is_empty()) {
            let extruder_areas = &extruder_area_opt.values;
            bed_poly = get_shared_poly(extruder_areas);
            if bed_poly.points.is_empty() {
                bed_poly.points = print_config_to_points(&bed_shape_opt.values);
            }
        } else {
            bed_poly.points = print_config_to_points(&bed_shape_opt.values);
        }
    } else {
        bed_poly.points = print_config_to_points(&bed_shape_opt.values);
    }

    bed_poly.points
}

// =============================================================================================
// Model.cpp:4744-4959 — free functions
// =============================================================================================

/// Model.cpp:4744-4752 — the same number of objects with the same IDs in the same order.
pub fn model_object_list_equal(model_old: &Model, model_new: &Model) -> bool {
    if model_old.objects.len() != model_new.objects.len() {
        return false;
    }
    for i in 0..model_old.objects.len() {
        if model_old.objects[i].id() != model_new.objects[i].id() {
            return false;
        }
    }
    true
}

/// Model.cpp:4756-4764 — the new model is the old one with objects added at the end.
pub fn model_object_list_extended(model_old: &Model, model_new: &Model) -> bool {
    if model_old.objects.len() >= model_new.objects.len() {
        return false;
    }
    for i in 0..model_old.objects.len() {
        if model_old.objects[i].id() != model_new.objects[i].id() {
            return false;
        }
    }
    true
}

/// Model.cpp:4766-4802 `model_volume_list_changed(old, new, TypeFilterFn type_filter)`
fn model_volume_list_changed_by(model_object_old: &ModelObject, model_object_new: &ModelObject, type_filter: &dyn Fn(ModelVolumeType) -> bool) -> bool {
    let (mut i_old, mut i_new) = (0usize, 0usize);
    while i_old < model_object_old.volumes.len() && i_new < model_object_new.volumes.len() {
        let mv_old = &model_object_old.volumes[i_old];
        let mv_new = &model_object_new.volumes[i_new];
        if !type_filter(mv_old.type_()) {
            i_old += 1;
            continue;
        }
        if !type_filter(mv_new.type_()) {
            i_new += 1;
            continue;
        }
        if mv_old.type_() != mv_new.type_() || mv_old.id() != mv_new.id() {
            return true;
        }
        //FIXME test for the content of the mesh!
        if !eigen_is_approx_mat4(&mv_old.matrix(), &mv_new.matrix()) {
            return true;
        }
        i_old += 1;
        i_new += 1;
    }
    while i_old < model_object_old.volumes.len() {
        let mv_old = &model_object_old.volumes[i_old];
        if type_filter(mv_old.type_()) {
            // ModelVolume was deleted.
            return true;
        }
        i_old += 1;
    }
    while i_new < model_object_new.volumes.len() {
        let mv_new = &model_object_new.volumes[i_new];
        if type_filter(mv_new.type_()) {
            // ModelVolume was added.
            return true;
        }
        i_new += 1;
    }
    false
}

/// Model.cpp:4804-4807 `model_volume_list_changed(old, new, const ModelVolumeType type)`
pub fn model_volume_list_changed(model_object_old: &ModelObject, model_object_new: &ModelObject, type_: ModelVolumeType) -> bool {
    model_volume_list_changed_by(model_object_old, model_object_new, &|t| t == type_)
}

/// Model.cpp:4809-4814 `model_volume_list_changed(old, new, const std::initializer_list<ModelVolumeType> &types)`
pub fn model_volume_list_changed_types(model_object_old: &ModelObject, model_object_new: &ModelObject, types: &[ModelVolumeType]) -> bool {
    model_volume_list_changed_by(model_object_old, model_object_new, &|t| types.contains(&t))
}

/// Model.cpp:4816-4839 `model_property_changed(old, new, type_filter, compare)`
pub fn model_property_changed(
    model_object_old: &ModelObject,
    model_object_new: &ModelObject,
    type_filter: &dyn Fn(ModelVolumeType) -> bool,
    compare: &dyn Fn(&ModelVolume, &ModelVolume) -> bool,
) -> bool {
    // assert(!model_volume_list_changed_by(model_object_old, model_object_new, type_filter)): compiled out in the reference build (-DNDEBUG)
    let (mut i_old, mut i_new) = (0usize, 0usize);
    while i_old < model_object_old.volumes.len() && i_new < model_object_new.volumes.len() {
        let mv_old = &model_object_old.volumes[i_old];
        let mv_new = &model_object_new.volumes[i_new];
        if !type_filter(mv_old.type_()) {
            i_old += 1;
            continue;
        }
        if !type_filter(mv_new.type_()) {
            i_new += 1;
            continue;
        }
        // assert(mv_old.type_() == mv_new.type_() && mv_old.id() == mv_new.id()): compiled out in the reference build (-DNDEBUG)
        if !compare(mv_old, mv_new) {
            return true;
        }
        i_old += 1;
        i_new += 1;
    }
    false
}

/// Model.cpp:4841-4846
pub fn model_custom_supports_data_changed(mo: &ModelObject, mo_new: &ModelObject) -> bool {
    model_property_changed(mo, mo_new, &|t| t == ModelVolumeType::ModelPart, &|mv_old, mv_new| {
        mv_old.supported_facets.timestamp_matches(&mv_new.supported_facets)
    })
}

/// Model.cpp:4848-4853
pub fn model_custom_fuzzy_skin_data_changed(mo: &ModelObject, mo_new: &ModelObject) -> bool {
    model_property_changed(mo, mo_new, &|t| t == ModelVolumeType::ModelPart, &|mv_old, mv_new| {
        mv_old.fuzzy_skin_facets.timestamp_matches(&mv_new.fuzzy_skin_facets)
    })
}

/// Model.cpp:4855-4860
pub fn model_custom_seam_data_changed(mo: &ModelObject, mo_new: &ModelObject) -> bool {
    model_property_changed(mo, mo_new, &|t| t == ModelVolumeType::ModelPart, &|mv_old, mv_new| {
        mv_old.seam_facets.timestamp_matches(&mv_new.seam_facets)
    })
}

/// Model.cpp:4862-4867
pub fn model_mmu_segmentation_data_changed(mo: &ModelObject, mo_new: &ModelObject) -> bool {
    model_property_changed(mo, mo_new, &|t| t == ModelVolumeType::ModelPart, &|mv_old, mv_new| {
        mv_old.mmu_segmentation_facets.timestamp_matches(&mv_new.mmu_segmentation_facets)
    })
}

/// Model.cpp:4869-4878
pub fn model_brim_points_data_changed(mo: &ModelObject, mo_new: &ModelObject) -> bool {
    if mo.brim_points.len() != mo_new.brim_points.len() {
        return true;
    }
    for i in 0..mo.brim_points.len() {
        if mo.brim_points[i] != mo_new.brim_points[i] {
            return true;
        }
    }
    false
}

/// Model.cpp:4880-4886
pub fn model_has_multi_part_objects(model: &Model) -> bool {
    for model_object in &model.objects {
        if model_object.volumes.len() != 1 || !model_object.volumes[0].is_model_part() {
            return true;
        }
    }
    false
}

/// Model.cpp:4888-4903
pub fn model_has_advanced_features(model: &Model) -> bool {
    let config_is_advanced = |config: &ModelConfig| {
        !(config.empty() || (config.size() == 1 && config.keys().first().map(|k| k == "extruder").unwrap_or(false)))
    };
    for model_object in &model.objects {
        // Is there more than one instance or advanced config data?
        if model_object.instances.len() > 1 || config_is_advanced(&model_object.config) {
            return true;
        }
        // Is there any modifier or advanced config data?
        for model_volume in &model_object.volumes {
            if !model_volume.is_model_part() || config_is_advanced(&model_volume.config) {
                return true;
            }
        }
    }
    false
}

/// Model.cpp:4907-4929 — the IDs of the model's objects, volumes, instances and materials are
/// valid and unique (the asserts, as a result).
pub fn check_model_ids_validity(model: &Model) -> bool {
    let mut ids = std::collections::BTreeSet::new();
    let mut ok = true;
    let mut check = |id: ObjectID| {
        ok &= id.valid();
        ok &= ids.insert(id);
    };
    for model_object in &model.objects {
        check(model_object.id());
        check(model_object.config.id());
        for model_volume in &model_object.volumes {
            check(model_volume.id());
            check(model_volume.config.id());
        }
        for model_instance in &model_object.instances {
            check(model_instance.id());
        }
    }
    for mm in model.materials.values() {
        check(mm.id());
        check(mm.config.id());
    }
    ok
}

/// Model.cpp:4931-4959 — the IDs of model1 and model2 match (the asserts, as a result).
pub fn check_model_ids_equal(model1: &Model, model2: &Model) -> bool {
    // Verify whether the IDs of model1 and model match.
    let mut ok = model1.objects.len() == model2.objects.len();
    if !ok {
        return false;
    }
    for idx_model in 0..model2.objects.len() {
        let model_object1 = &model1.objects[idx_model];
        let model_object2 = &model2.objects[idx_model];
        ok &= model_object1.id() == model_object2.id();
        ok &= model_object1.config.id() == model_object2.config.id();
        ok &= model_object1.volumes.len() == model_object2.volumes.len();
        ok &= model_object1.instances.len() == model_object2.instances.len();
        if !ok {
            return false;
        }
        for i in 0..model_object1.volumes.len() {
            ok &= model_object1.volumes[i].id() == model_object2.volumes[i].id();
            ok &= model_object1.volumes[i].config.id() == model_object2.volumes[i].config.id();
        }
        for i in 0..model_object1.instances.len() {
            ok &= model_object1.instances[i].id() == model_object2.instances[i].id();
        }
    }
    ok &= model1.materials.len() == model2.materials.len();
    if ok {
        for ((k1, m1), (k2, m2)) in model1.materials.iter().zip(model2.materials.iter()) {
            ok &= k1 == k2; // compare keys
            ok &= m1.id() == m2.id();
            ok &= m1.config.id() == m2.config.id();
        }
    }
    ok
}

// =============================================================================================
// Loaders' members
// =============================================================================================

#[derive(Debug, Clone, Default)]
/// Model metadata for descriptive information.
pub struct ModelMetadata {
    /// Model name/title.
    pub title: Option<String>,
    /// Author/creator.
    pub author: Option<String>,
    /// Description text.
    pub description: Option<String>,
    /// Creation timestamp.
    pub created_at: Option<String>,
    /// Modification timestamp.
    pub modified_at: Option<String>,
    /// Application that created the model.
    pub application: Option<String>,
}

#[derive(Debug, Clone, Default)]
/// Per-object configuration overrides of the earlier loaders (no longer a member of
/// `ModelObject`, whose `config` is the C++ `ModelConfigObject`).
pub struct ObjectConfig {
    /// Layer height override.
    pub layer_height: Option<f64>,
    /// Infill density override.
    pub infill_density: Option<f64>,
    /// Number of perimeters override.
    pub perimeters: Option<u32>,
    /// Extruder to use for this object.
    pub extruder: Option<u32>,
}

/// Load a model from a file (detects format by extension).
/// Model.cpp:306-489 (the formats the crate reads)
pub fn load_model(path: &PathBuf) -> crate::Result<Model> {
    let ext = path
        .extension()
        .and_then(|e| e.to_str())
        .map(|e| e.to_lowercase());

    match ext.as_deref() {
        Some("stl") => {
            let mesh = crate::stl::read_stl_file(path)?;
            let mut model = Model::new();
            let object = ModelObject::new("object", mesh);
            model.add_object(object);
            model.file_path = Some(path.clone());
            Ok(model)
        }
        Some("obj") => {
            let loaded = crate::obj::load_obj(path)?;
            let mut model = Model::new();
            for obj in loaded.objects.into_iter() {
                model.add_object(obj);
            }
            model.file_path = Some(path.clone());
            Ok(model)
        }
        Some("3mf") => {
            let mut model = Model::new();
            let mut config = crate::calib::DynamicPrintConfig::default();
            let mut config_substitutions =
                crate::format::bbs_3mf::ConfigSubstitutionContext::default();
            let input_file = path.to_string_lossy();
            let result = crate::format::three_mf::load_3mf(
                &input_file,
                &mut config,
                &mut config_substitutions,
                &mut model,
                true,
            )?;
            if !result {
                return Err(crate::Error::Mesh(
                    "Loading of a model file failed.".to_string(),
                ));
            }
            if model.objects.is_empty() {
                return Err(crate::Error::Mesh(
                    "The supplied file couldn't be read because it's empty".to_string(),
                ));
            }
            model.file_path = Some(path.clone());
            Ok(model)
        }
        _ => {
            Err(crate::Error::Mesh(format!(
                "Unsupported file format: {:?}",
                ext
            )))
        }
    }
}

#[allow(dead_code)]
fn _unused_legacy_bbox(_b: BoundingBox3F) {}
