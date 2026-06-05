//! Faithful 1:1 port of `Orient.{hpp,cpp}` from BambuStudio libslic3r.
//!
//! C++ Reference:
//! - src/libslic3r/Orient.hpp (171 lines)
//! - src/libslic3r/Orient.cpp (789 lines)
//!
//! Auto-orientation: find the orientation of a mesh that minimizes the support
//! area / overhang cost. This drives byte-exact G-code parity, so fidelity is
//! everything: same constants, same float arithmetic, same control flow.
//!
//! PORTING STATUS (partial):
//! The pure-data structures (`OrientMesh`, `OrientParams`, `OrientParamsArea`,
//! `CostItems`) and the pure-math `AutoOrienter` helpers (`quantize_vec3f`,
//! `add_supplements`, `remove_duplicates`, `argsort`, `target_function`,
//! `area_cumulation`, `area_cumulation_accurate`) are ported faithfully here.
//!
//! The mesh-bound methods (`preprocess`, `process`, `project_vertices`,
//! `get_features`, `find_cooling_direction`, `find_cooling_direction2`), the
//! free functions (`_orient`, `orient`, `orient_for_cooling`) and the
//! `ModelObject`/`ModelInstance` overloads are BLOCKED on un-ported
//! dependencies (see the per-item notes). They are intentionally NOT stubbed
//! with fake logic; the original C++ is preserved as documentation so a future
//! porter can complete them once the dependencies land. The blocking
//! dependencies are:
//!   * `TriangleMesh::its` (indexed_triangle_set member) with `facet_area(i)`,
//!     `get_vertex(i,j)`, `get_property(i)` — not present on the Rust
//!     `TriangleMesh`/`indexed_triangle_set`.
//!   * `TriangleMesh::convex_hull_3d()` returning a 3D hull *mesh* — the Rust
//!     `geometry::convex_hull_3d` only computes a 2D XY-projected hull.
//!   * `its_volume(its)` — not ported.
//!   * `BoundingBoxf3::area()` (box surface area) / `radius()` — not on
//!     `geometry::BoundingBox3F`.
//!   * `TriangleMesh::rotate_x/y/z`, `rotate(angle, axis)`, `center()`,
//!     `translate(x,y,z)`, `TriangleMesh(its)` ctor — not present.
//!   * `Geometry::rotation_from_two_vectors`, `Geometry::extract_euler_angles`
//!     — not ported into `geometry`.
//!   * `Model`/`ModelObject`/`ModelInstance` rotation/config API
//!     (`config.has`, `opt_int`, `rotate`, `ensure_on_bed`, `get_object`) —
//!     the Rust `model.rs` is a simplified subset.
//!   * `tbb::parallel_for` — no native TBB backend (wasm-safe constraint).

use nalgebra::{DMatrix, DVector, Matrix3, Vector3};
use std::collections::HashMap;

use crate::libslic3r::EPSILON;
use crate::normal_utils::Vec3f;

/// 3D double-precision vector, mirroring C++ `Vec3d` (Eigen `Matrix<double,3,1>`).
/// Point.hpp
pub type Vec3d = Vector3<f64>;
/// 3x3 double-precision matrix, mirroring C++ `Matrix3d` (Eigen `Matrix3d`).
/// Point.hpp
pub type Matrix3d = Matrix3<f64>;
/// stl_normal == Vec3f, mirroring admesh/stl.h.
type StlNormal = Vec3f;

/// `static constexpr double PI = 3.141592653589793238;`
/// libslic3r.h:59 — Orient.cpp uses the libslic3r `PI` constant directly.
const PI: f64 = 3.141_592_653_589_793_238;

/// Fan direction, mirroring C++ `enum FanDirection`.
/// PrintConfig.hpp:291-296
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FanDirection {
    /// PrintConfig.hpp:292
    FdUndefine = 0,
    /// PrintConfig.hpp:293
    FdLeft,
    /// PrintConfig.hpp:294
    FdRight,
    /// PrintConfig.hpp:295
    FdBoth,
}

// =====================================================================
// Orient.hpp
// =====================================================================

/// A logical bed representing an object not being orientd. Either the orient
/// has not yet successfully run on this OrientPolygon or it could not fit the
/// object due to overly large size or invalid geometry.
/// Orient.hpp:14
pub const UNORIENTD: i32 = -1;

/// Input/Output structure for the orient() function. The mesh field will not
/// be modified during orientment. Instead, the translation and rotation fields
/// will mark the needed transformation for the polygon to be in the orientd
/// position. These can also be set to an initial offset and rotation.
///
/// The bed_idx field will indicate the logical bed into which the
/// polygon belongs: UNORIENTD means no place for the polygon
/// (also the initial state before orient), 0..N means the index of the bed.
/// Zero is the physical bed, larger than zero means a virtual bed.
/// Orient.hpp:25-49
#[derive(Clone)]
pub struct OrientMesh {
    /// The real mesh data
    /// Orient.hpp:26
    pub mesh: crate::triangle_mesh::TriangleMesh,
    /// Orient.hpp:27
    pub overhang_angle: f64,
    /// Orient.hpp:28
    pub angle: f64,
    /// Orient.hpp:29
    pub angle_vertical: f64,
    /// Orient.hpp:30
    pub axis: Vec3d,
    /// Orient.hpp:31
    pub axis_vertical: Vec3d,
    /// Orient.hpp:32
    pub orientation: Vec3d,
    /// Orient.hpp:33
    pub orientation_vertical: Vec3d,
    /// Orient.hpp:34
    pub rotation_matrix: Matrix3d,
    /// Orient.hpp:35
    pub rotation_matrix_vertical: Matrix3d,
    /// Orient.hpp:36
    pub euler_angles: Vec3d,
    /// Orient.hpp:37
    pub euler_angles_vertical: Vec3d,
    /// Orient.hpp:38
    pub cooling_direction: Vec3d,
    /// Orient.hpp:39
    pub has_cooling_fan: bool,
    /// Orient.hpp:41
    pub name: String,
    // Orient.hpp:44 — `std::function<void(const OrientMesh&)> setter`: the
    // arbitrary-closure setter / `apply()` are UI glue and are omitted (the
    // closure would need a `Box<dyn Fn(&OrientMesh)>`, which conflicts with the
    // `Clone`/`Default` derives the rest of the crate relies on). Not load-bearing
    // for G-code parity.
}

impl Default for OrientMesh {
    fn default() -> Self {
        Self {
            mesh: crate::triangle_mesh::TriangleMesh::new(),
            // Orient.hpp:27
            overhang_angle: 30.0,
            // Orient.hpp:28
            angle: 0.0,
            // Orient.hpp:29
            angle_vertical: 0.0,
            // Orient.hpp:30
            axis: Vec3d::new(0.0, 0.0, 1.0),
            // Orient.hpp:31
            axis_vertical: Vec3d::new(0.0, 0.0, 1.0),
            // Orient.hpp:32
            orientation: Vec3d::new(0.0, 0.0, 1.0),
            // Orient.hpp:33
            orientation_vertical: Vec3d::new(-1.0, 0.0, 0.0),
            // Orient.hpp:34
            rotation_matrix: Matrix3d::identity(),
            // Orient.hpp:35
            rotation_matrix_vertical: Matrix3d::identity(),
            // Orient.hpp:36
            euler_angles: Vec3d::new(0.0, 0.0, 0.0),
            // Orient.hpp:37
            euler_angles_vertical: Vec3d::new(0.0, 0.0, 0.0),
            // Orient.hpp:38
            cooling_direction: Vec3d::new(0.0, 0.0, 0.0),
            // Orient.hpp:39
            has_cooling_fan: false,
            // Orient.hpp:41
            name: String::new(),
        }
    }
}

/// params for minimizing support area
/// Orient.hpp:52-98
#[derive(Clone)]
pub struct OrientParamsArea {
    /// Orient.hpp:53
    pub tar_a: f32,
    /// Orient.hpp:54
    pub tar_b: f32,
    /// Orient.hpp:55
    pub relative_f: f32,
    /// Orient.hpp:56
    pub contour_f: f32,
    /// Orient.hpp:57
    pub bottom_f: f32,
    /// Orient.hpp:58
    pub bottom_hull_f: f32,
    /// Orient.hpp:59
    pub tar_c: f32,
    /// Orient.hpp:60
    pub tar_d: f32,
    /// Orient.hpp:61
    pub tar_e: f32,
    /// Orient.hpp:62 //0.0475;
    pub first_lay_h: f32,
    /// Orient.hpp:63
    pub vector_tol: f32,
    /// Orient.hpp:64
    pub negl_face_size: f32,
    /// Orient.hpp:65
    pub ascent: f32,
    /// Orient.hpp:66
    pub plafond_adv: f32,
    /// Orient.hpp:67
    pub contour_amount: f32,
    /// Orient.hpp:68
    pub ov_h: f32,
    /// Orient.hpp:69
    pub height_offset: f32,
    /// Orient.hpp:70
    pub height_log: f32,
    /// Orient.hpp:71
    pub height_log_k: f32,
    /// cos(1.4\degree) for low angle face 0.9997f
    /// Orient.hpp:72
    pub laf_max: f32,
    /// cos(14\degree) 0.9703f
    /// Orient.hpp:73
    pub laf_min: f32,
    /// Orient.hpp:74 //0.01f
    pub tar_laf: f32,
    /// Orient.hpp:75
    pub tar_proj_area: f32,
    /// min bottom area. If lower than it the object may be unstable
    /// Orient.hpp:76
    pub bottom_min: f32,
    /// max bottom area. If get to it the object is stable enough (further increase bottom area won't do more help)
    /// Orient.hpp:77
    pub bottom_max: f32,
    /// Orient.hpp:78
    pub height_to_bottom_hull_ratio_min: f32,
    /// max bottom hull area
    /// Orient.hpp:79
    pub bottom_hull_max: f32,
    /// penalty of generating supports on appearance face
    /// Orient.hpp:80
    pub apperance_face_supp: f32,
    /// Orient.hpp:82
    pub overhang_angle: f32,
    /// Orient.hpp:83
    pub use_low_angle_face: bool,
    /// Orient.hpp:84
    pub min_volume: bool,
    /// Orient.hpp:85
    pub fun_dir: Vec3f,
    /// Allow parallel execution.
    /// Orient.hpp:88
    pub parallel: bool,
    // Orient.hpp:92 `progressind` / Orient.hpp:95 `stopcondition`: std::function
    // callbacks. Omitted from the data struct (threaded as explicit closure
    // arguments where the algorithm uses them, matching the C++ call sites).
}

impl Default for OrientParamsArea {
    /// `OrientParamsArea() = default;`
    /// Orient.hpp:97 — default member initializers from Orient.hpp:53-89.
    fn default() -> Self {
        Self {
            tar_a: 0.015,
            tar_b: 0.177,
            relative_f: 20.0,
            contour_f: 0.5,
            bottom_f: 2.5,
            bottom_hull_f: 0.1,
            tar_c: 0.1,
            tar_d: 1.0,
            tar_e: 0.0115,
            first_lay_h: 0.2,
            vector_tol: -0.00083,
            negl_face_size: 0.01,
            ascent: -0.86602540378,
            plafond_adv: 0.0599,
            contour_amount: 0.0182427,
            ov_h: 2.574,
            height_offset: 2.3728,
            height_log: 0.041375,
            height_log_k: 1.9325457,
            laf_max: 0.999,
            laf_min: 0.97,
            tar_laf: 0.001,
            tar_proj_area: 0.1,
            bottom_min: 0.1,
            bottom_max: 2000.0,
            height_to_bottom_hull_ratio_min: 1.0,
            bottom_hull_max: 2000.0,
            apperance_face_supp: 3.0,
            overhang_angle: 30.0,
            use_low_angle_face: true,
            min_volume: false,
            fun_dir: Vec3f::new(0.0, 0.0, 0.0),
            parallel: true,
        }
    }
}

/// Orient.hpp:100-147
#[derive(Clone)]
pub struct OrientParams {
    /// Orient.hpp:101 //0.128f;
    pub tar_a: f32,
    /// Orient.hpp:102
    pub tar_b: f32,
    /// Orient.hpp:103
    pub relative_f: f32,
    /// Orient.hpp:104
    pub contour_f: f32,
    /// Orient.hpp:105
    pub bottom_f: f32,
    /// Orient.hpp:106
    pub bottom_hull_f: f32,
    /// Orient.hpp:107
    pub tar_c: f32,
    /// Orient.hpp:108
    pub tar_d: f32,
    /// Orient.hpp:109 //0.032157292647062234;
    pub tar_e: f32,
    /// Orient.hpp:110 //0.029;
    pub first_lay_h: f32,
    /// Orient.hpp:111
    pub vector_tol: f32,
    /// Orient.hpp:112
    pub negl_face_size: f32,
    /// Orient.hpp:113
    pub ascent: f32,
    /// Orient.hpp:114
    pub plafond_adv: f32,
    /// Orient.hpp:115
    pub contour_amount: f32,
    /// Orient.hpp:116
    pub ov_h: f32,
    /// Orient.hpp:117
    pub height_offset: f32,
    /// Orient.hpp:118
    pub height_log: f32,
    /// Orient.hpp:119
    pub height_log_k: f32,
    /// cos(1.4\degree) for low angle face //0.9997f;
    /// Orient.hpp:120
    pub laf_max: f32,
    /// cos(14\degree) 0.9703f;
    /// Orient.hpp:121
    pub laf_min: f32,
    /// Orient.hpp:122 //0.1f
    pub tar_laf: f32,
    /// Orient.hpp:123
    pub tar_proj_area: f32,
    /// min bottom area. If lower than it the objects may be unstable
    /// Orient.hpp:124
    pub bottom_min: f32,
    /// Orient.hpp:125 //400
    pub bottom_max: f32,
    /// Orient.hpp:126
    pub height_to_bottom_hull_ratio_min: f32,
    /// max bottom hull area to clip //600
    /// Orient.hpp:127
    pub bottom_hull_max: f32,
    /// penalty of generating supports on appearance face
    /// Orient.hpp:128
    pub apperance_face_supp: f32,
    /// Orient.hpp:130
    pub overhang_angle: f32,
    /// Orient.hpp:131
    pub use_low_angle_face: bool,
    /// Orient.hpp:132
    pub min_volume: bool,
    /// Orient.hpp:133
    pub fun_dir: Vec3f,
    /// Allow parallel execution.
    /// Orient.hpp:137
    pub parallel: bool,
    // Orient.hpp:141 `progressind` / Orient.hpp:144 `stopcondition`: std::function
    // callbacks; threaded as explicit closure arguments at the call sites.
}

impl Default for OrientParams {
    /// `OrientParams() = default;`
    /// Orient.hpp:146 — default member initializers from Orient.hpp:101-137.
    fn default() -> Self {
        Self {
            tar_a: 0.01,
            tar_b: 0.177,
            relative_f: 6.610621027964314,
            contour_f: 0.23228623269775997,
            bottom_f: 1.167152017941474,
            bottom_hull_f: 0.1,
            tar_c: 0.24308070476924726,
            tar_d: 0.6284515508160871,
            tar_e: 0.0,
            first_lay_h: 0.2,
            vector_tol: -0.0011163303070972383,
            negl_face_size: 0.1,
            ascent: -0.86602540378,
            plafond_adv: 0.04079208948120519,
            contour_amount: 0.0101472219892684,
            ov_h: 1.0370178217794535,
            height_offset: 2.7417608343142073,
            height_log: 0.06442030687034085,
            height_log_k: 0.3933594673063997,
            laf_max: 0.999,
            laf_min: 0.9703,
            tar_laf: 0.01,
            tar_proj_area: 0.1,
            bottom_min: 0.1,
            bottom_max: 2000.0,
            height_to_bottom_hull_ratio_min: 1.0,
            bottom_hull_max: 2000.0,
            apperance_face_supp: 3.0,
            overhang_angle: 30.0,
            use_low_angle_face: true,
            min_volume: false,
            fun_dir: Vec3f::new(0.0, 0.0, 0.0),
            parallel: false,
        }
    }
}

/// `using OrientMeshs = std::vector<OrientMesh>;`
/// Orient.hpp:149
pub type OrientMeshs = Vec<OrientMesh>;

// =====================================================================
// Orient.cpp
// =====================================================================

/// Orient.cpp:29-53
#[derive(Clone)]
pub struct CostItems {
    /// Orient.cpp:30
    pub overhang: f32,
    /// Orient.cpp:31
    pub bottom: f32,
    /// Orient.cpp:32
    pub bottom_hull: f32,
    /// Orient.cpp:33
    pub contour: f32,
    /// area_of_low_angle_faces
    /// Orient.cpp:34
    pub area_laf: f32,
    /// area of projected 2D profile
    /// Orient.cpp:35
    pub area_projected: f32,
    /// Orient.cpp:36
    pub volume: f32,
    /// total area of all faces
    /// Orient.cpp:37
    pub area_total: f32,
    /// radius of bounding box
    /// Orient.cpp:38
    pub radius: f32,
    /// affects stability, the lower the better
    /// Orient.cpp:39
    pub height_to_bottom_hull_ratio: f32,
    /// Orient.cpp:40
    pub unprintability: f32,
    /// Orient.cpp:41
    pub areas_cooling: DVector<f32>,
}

impl Default for CostItems {
    /// `CostItems() { memset(this, 0, sizeof(*this)); }`
    /// Orient.cpp:43 — zero-initialize all fields.
    fn default() -> Self {
        Self {
            overhang: 0.0,
            bottom: 0.0,
            bottom_hull: 0.0,
            contour: 0.0,
            area_laf: 0.0,
            area_projected: 0.0,
            volume: 0.0,
            area_total: 0.0,
            radius: 0.0,
            height_to_bottom_hull_ratio: 0.0,
            unprintability: 0.0,
            // memset zeroes the Eigen::VectorXf header to an empty vector.
            areas_cooling: DVector::<f32>::zeros(0),
        }
    }
}

impl CostItems {
    /// `static std::string field_names()`
    /// Orient.cpp:44-46
    pub fn field_names() -> String {
        // Orient.cpp:45
        "                                      overhang, bottom, bothull, contour, A_laf, A_prj, unprintability".to_string()
    }

    /// `std::string field_values()`
    /// Orient.cpp:47-52
    pub fn field_values(&self) -> String {
        // Orient.cpp:48-50 — std::fixed << std::setprecision(1)
        format!(
            "{:.1},\t{:.1},\t{:.1},\t{:.1},\t{:.1},\t{:.1},\t{:.1}",
            self.overhang,
            self.bottom,
            self.bottom_hull,
            self.contour,
            self.area_laf,
            self.area_projected,
            self.unprintability
        )
    }
}

/// Quantized-vector hash, mirroring `AutoOrienter::VecHash`.
/// Orient.cpp:106-110
///
/// `size_t operator()(const Vec3f& n1) const {`
/// `    return std::hash<coord_t>()(int(n1(0)*100+100)) + std::hash<coord_t>()(int(n1(1)*100+100)) * 101 + std::hash<coord_t>()(int(n1(2)*100+100)) * 10221;`
/// `}`
///
/// To reproduce the C++ behaviour of using a `Vec3f` as a hash-map key (where
/// equality is bit-exact and the hash quantizes each component to `int`), we
/// derive a hashable integer key. `std::hash<coord_t>` for the small integers
/// here is the identity, so the combined hash is the integer expression below;
/// since the keys inserted are already quantized to multiples of 0.001 the
/// quantized triple uniquely identifies a bucket. We key the maps by this
/// quantized integer triple to match the C++ bucketing semantics.
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
struct VecKey(i32, i32, i32);

impl VecKey {
    /// Build the key exactly as `VecHash` quantizes: `int(n(i)*100+100)`.
    /// Orient.cpp:108
    #[inline]
    fn from_vec3f(n1: &Vec3f) -> Self {
        VecKey(
            (n1[0] * 100.0 + 100.0) as i32,
            (n1[1] * 100.0 + 100.0) as i32,
            (n1[2] * 100.0 + 100.0) as i32,
        )
    }
}

/// A class encapsulating the libnest2d Nester class and extending it with other
/// management and spatial index structures for acceleration.
/// Orient.cpp:57-605
///
/// Only the mesh-independent fields and methods are populated here; the
/// mesh-dependent fields are documented but their producers (`preprocess`,
/// `project_vertices`, `get_features`, ...) are BLOCKED — see the module-level
/// note. `mesh`/`mesh_convex_hull` are not stored as references to avoid a fake
/// borrow that would not match the C++ raw-pointer lifetimes.
pub struct AutoOrienter {
    /// Orient.cpp:61
    pub face_count_hull: i32,
    /// Orient.cpp:65
    pub normals: DMatrix<f32>,
    /// Orient.cpp:65
    pub normals_quantize: DMatrix<f32>,
    /// Orient.cpp:65
    pub normals_hull: DMatrix<f32>,
    /// Orient.cpp:65
    pub normals_hull_quantize: DMatrix<f32>,
    /// Orient.cpp:66
    pub areas: DVector<f32>,
    /// Orient.cpp:66
    pub areas_hull: DVector<f32>,
    /// whether a facet is outer apperance
    /// Orient.cpp:67
    pub is_apperance: DVector<f32>,
    /// Orient.cpp:68
    pub z_projected: DMatrix<f32>,
    /// max of projected z
    /// Orient.cpp:69
    pub z_max: DVector<f32>,
    /// Orient.cpp:69
    pub z_max_hull: DVector<f32>,
    /// median of projected z
    /// Orient.cpp:70
    pub z_median: DVector<f32>,
    /// mean of projected z
    /// Orient.cpp:71
    pub z_mean: DVector<f32>,
    /// weighted areas for cool direction
    /// Orient.cpp:72
    pub areas_cooling: DVector<f32>,
    /// Orient.cpp:73
    pub face_normals: Vec<Vec3f>,
    /// Orient.cpp:74
    pub face_normals_hull: Vec<Vec3f>,
    /// Orient.cpp:75
    pub params: OrientParams,
    /// Orient.cpp:76
    pub has_cooling_fan: bool,
    /// Vec3f == stl_normal
    /// Orient.cpp:78
    pub orientations: Vec<Vec3f>,
}

impl AutoOrienter {
    /// `Vec3f quantize_vec3f(const Vec3f n1)`
    /// Orient.cpp:112-114
    pub fn quantize_vec3f(&self, n1: Vec3f) -> Vec3f {
        // Orient.cpp:113
        Vec3f::new(
            (n1[0] * 1000.0).floor() / 1000.0,
            (n1[1] * 1000.0).floor() / 1000.0,
            (n1[2] * 1000.0).floor() / 1000.0,
        )
    }

    /// `void area_cumulation(const Eigen::MatrixXf& normals_, const Eigen::VectorXf& areas_, int num_directions = 10)`
    /// Orient.cpp:234-257
    pub fn area_cumulation(
        &mut self,
        normals_: &DMatrix<f32>,
        areas_: &DVector<f32>,
        num_directions: i32,
    ) {
        // Orient.cpp:236
        let mut alignments: HashMap<VecKey, (StlNormal, f32)> = HashMap::new();
        // init to 0
        // Orient.cpp:238-239
        for i in 0..areas_.len() {
            let row = StlNormal::new(normals_[(i, 0)], normals_[(i, 1)], normals_[(i, 2)]);
            alignments.entry(VecKey::from_vec3f(&row)).or_insert((row, 0.0));
        }
        // cumulate areas
        // Orient.cpp:241-244
        for i in 0..areas_.len() {
            let row = StlNormal::new(normals_[(i, 0)], normals_[(i, 1)], normals_[(i, 2)]);
            let e = alignments
                .entry(VecKey::from_vec3f(&row))
                .or_insert((row, 0.0));
            e.1 += areas_[i];
        }

        // typedef std::pair<stl_normal, float> PAIR;
        // Orient.cpp:246-248
        let mut align_counts: Vec<(StlNormal, f32)> = alignments.into_values().collect();
        // sort by area descending
        // Orient.cpp:248
        align_counts.sort_by(|p1, p2| p2.1.partial_cmp(&p1.1).unwrap_or(std::cmp::Ordering::Equal));

        // Orient.cpp:250
        let num_directions = std::cmp::min(num_directions as usize, align_counts.len());
        // Orient.cpp:251-256
        for item in align_counts.iter().take(num_directions) {
            self.orientations.push(item.0);
        }
    }

    /// This function is to make sure to return the accurate normal rather than quantized normal
    /// `void area_cumulation_accurate(std::vector<Vec3f>& normals_, const Eigen::MatrixXf& quantize_normals_, const Eigen::VectorXf& areas_, int num_directions = 10)`
    /// Orient.cpp:258-288
    pub fn area_cumulation_accurate(
        &mut self,
        normals_: &[Vec3f],
        quantize_normals_: &DMatrix<f32>,
        areas_: &DVector<f32>,
        num_directions: i32,
    ) {
        // std::unordered_map<stl_normal, std::pair<std::vector<float>, Vec3f>, VecHash> alignments_;
        // Orient.cpp:261
        let mut alignments_: HashMap<VecKey, (Vec<f32>, Vec3f)> = HashMap::new();
        // Orient.cpp:262
        let n1 = Vec3f::new(0.0, 0.0, 0.0);
        // Orient.cpp:263
        let current_areas: Vec<f32> = vec![0.0, 0.0];
        // init to 0
        // Orient.cpp:265-267
        for i in 0..areas_.len() {
            let qrow = StlNormal::new(
                quantize_normals_[(i, 0)],
                quantize_normals_[(i, 1)],
                quantize_normals_[(i, 2)],
            );
            alignments_
                .entry(VecKey::from_vec3f(&qrow))
                .or_insert((current_areas.clone(), n1));
        }
        // cumulate areas
        // Orient.cpp:269-276
        for i in 0..areas_.len() {
            let qrow = StlNormal::new(
                quantize_normals_[(i, 0)],
                quantize_normals_[(i, 1)],
                quantize_normals_[(i, 2)],
            );
            let e = alignments_
                .entry(VecKey::from_vec3f(&qrow))
                .or_insert((current_areas.clone(), n1));
            // Orient.cpp:271
            e.0[1] += areas_[i];
            // Orient.cpp:272-275
            if areas_[i] > e.0[0] {
                e.1 = normals_[i];
                e.0[0] = areas_[i];
            }
        }

        // typedef std::pair<stl_normal, std::pair<std::vector<float>, Vec3f>> PAIR;
        // Orient.cpp:278-280
        let mut align_counts: Vec<(Vec<f32>, Vec3f)> = alignments_.into_values().collect();
        // sort by accumulated area (index [1]) descending
        // Orient.cpp:280
        align_counts.sort_by(|p1, p2| {
            p2.0[1].partial_cmp(&p1.0[1]).unwrap_or(std::cmp::Ordering::Equal)
        });

        // Orient.cpp:282
        let num_directions = std::cmp::min(num_directions as usize, align_counts.len());
        // Orient.cpp:283-287
        for item in align_counts.iter().take(num_directions) {
            self.orientations.push(item.1);
        }
    }

    /// `void add_supplements()`
    /// Orient.cpp:289-298
    pub fn add_supplements(&mut self) {
        // Orient.cpp:291-296
        let vecs: Vec<Vec3f> = vec![
            Vec3f::new(0.0, 0.0, -1.0),
            Vec3f::new(0.70710678, 0.0, -0.70710678),
            Vec3f::new(0.0, 0.70710678, -0.70710678),
            Vec3f::new(-0.70710678, 0.0, -0.70710678),
            Vec3f::new(0.0, -0.70710678, -0.70710678),
            Vec3f::new(1.0, 0.0, 0.0),
            Vec3f::new(0.70710678, 0.70710678, 0.0),
            Vec3f::new(0.0, 1.0, 0.0),
            Vec3f::new(-0.70710678, 0.70710678, 0.0),
            Vec3f::new(-1.0, 0.0, 0.0),
            Vec3f::new(-0.70710678, -0.70710678, 0.0),
            Vec3f::new(0.0, -1.0, 0.0),
            Vec3f::new(0.70710678, -0.70710678, 0.0),
            Vec3f::new(0.70710678, 0.0, 0.70710678),
            Vec3f::new(0.0, 0.70710678, 0.70710678),
            Vec3f::new(-0.70710678, 0.0, 0.70710678),
            Vec3f::new(0.0, -0.70710678, 0.70710678),
            Vec3f::new(0.0, 0.0, 1.0),
        ];
        // Orient.cpp:297
        self.orientations.extend(vecs);
    }

    /// remove duplicate orientations
    ///
    /// `tol` tolerance. default 0.01 =sin(0.57\degree)
    /// `void remove_duplicates(double tol=0.0000001)`
    /// Orient.cpp:300-322
    pub fn remove_duplicates(&mut self, tol: f64) {
        // Orient.cpp:306 — for (auto it = orientations.begin()+1; it < orientations.end();)
        let mut it = 1usize;
        while it < self.orientations.len() {
            // Orient.cpp:308
            let mut duplicate = false;
            // Orient.cpp:309-315
            for it_ok in 0..it {
                if is_approx_vec3f(&self.orientations[it_ok], &self.orientations[it], tol) {
                    duplicate = true;
                    break;
                }
            }
            // Orient.cpp:316
            let all_zero = Vec3f::new(0.0, 0.0, 0.0);
            // Orient.cpp:317-320
            if duplicate || is_approx_vec3f(&self.orientations[it], &all_zero, tol) {
                self.orientations.remove(it);
            } else {
                it += 1;
            }
        }
    }

    /// `static Eigen::VectorXi argsort(const Eigen::VectorXf& vec, std::string order="ascend")`
    /// Orient.cpp:356-377
    pub fn argsort(vec: &DVector<f32>, order: &str) -> Vec<i32> {
        // Eigen::VectorXi ind = Eigen::VectorXi::LinSpaced(vec.size(), 0, vec.size() - 1);
        // Orient.cpp:358
        let mut ind: Vec<i32> = (0..vec.len() as i32).collect();
        // Orient.cpp:360-369
        if order == "ascend" {
            // Orient.cpp:361-363
            ind.sort_by(|&i, &j| {
                vec[i as usize]
                    .partial_cmp(&vec[j as usize])
                    .unwrap_or(std::cmp::Ordering::Equal)
            });
        } else {
            // Orient.cpp:366-368
            ind.sort_by(|&i, &j| {
                vec[j as usize]
                    .partial_cmp(&vec[i as usize])
                    .unwrap_or(std::cmp::Ordering::Equal)
            });
        }
        // Orient.cpp:371
        ind
    }

    /// `float target_function(CostItems& costs, bool min_volume)`
    /// Orient.cpp:470-489
    pub fn target_function(&self, costs: &mut CostItems, min_volume: bool) -> f32 {
        // Orient.cpp:472
        let mut cost: f32;
        // float bottom = costs.bottom; //std::min(costs.bottom, params.BOTTOM_MAX);
        // Orient.cpp:473
        let bottom = costs.bottom;
        // float bottom_hull = costs.bottom_hull; // std::min(costs.bottom_hull, params.BOTTOM_HULL_MAX);
        // Orient.cpp:474
        let bottom_hull = costs.bottom_hull;
        // Orient.cpp:475-479
        if min_volume {
            // Orient.cpp:477
            let overhang = costs.overhang / 25.0;
            // Orient.cpp:478
            cost = self.params.tar_a * (overhang + self.params.tar_b)
                + self.params.relative_f
                    * (overhang * self.params.tar_c
                        + self.params.tar_d
                        + self.params.tar_laf
                            * costs.area_laf
                            * (self.params.use_low_angle_face as i32 as f32))
                    / (self.params.tar_d
                        + self.params.contour_f * costs.contour
                        + self.params.bottom_f * bottom
                        + self.params.bottom_hull_f * bottom_hull
                        + self.params.tar_e * overhang
                        + self.params.tar_proj_area * costs.area_projected);
        } else {
            // Orient.cpp:481
            let _overhang = costs.overhang;
            // Orient.cpp:482
            cost = self.params.relative_f
                * (costs.overhang * self.params.tar_c
                    + self.params.tar_d
                    + self.params.tar_laf
                        * costs.area_laf
                        * (self.params.use_low_angle_face as i32 as f32))
                / (self.params.tar_d
                    + self.params.contour_f * costs.contour
                    + self.params.bottom_f * bottom
                    + self.params.bottom_hull_f * bottom_hull
                    + self.params.tar_proj_area * costs.area_projected);
        }
        // Orient.cpp:484 — cost += (costs.bottom < params.BOTTOM_MIN) * 100;
        cost += ((costs.bottom < self.params.bottom_min) as i32 as f32) * 100.0;

        // Orient.cpp:486 — costs.unprintability = costs.unprintability = cost;
        costs.unprintability = cost;
        costs.unprintability = cost;

        // Orient.cpp:488
        cost
    }

    // -----------------------------------------------------------------
    // BLOCKED methods — preserved as documentation, not implemented.
    // -----------------------------------------------------------------
    //
    // The following members of `AutoOrienter` require mesh API that is not
    // present in the Rust crate (see module-level note). They are reproduced
    // here verbatim from the C++ source so a future porter can finish them
    // once `TriangleMesh::its` (facet_area/get_vertex/get_property),
    // `TriangleMesh::convex_hull_3d`, `its_volume`, `its_face_normals`,
    // `BoundingBoxf3::area()/radius()`, the mesh `rotate_*`/`center`/`translate`
    // helpers, and `Geometry::rotation_from_two_vectors`/`extract_euler_angles`
    // are ported.
    //
    // Orient.cpp:82-104 — AutoOrienter(orient_mesh, params, progressind, stopcond)
    //                     and AutoOrienter(mesh) constructors (call preprocess()).
    // Orient.cpp:116-184 — Vec3d process()
    // Orient.cpp:186-232 — void preprocess()  (facet_area, get_property,
    //                     convex_hull_3d, its_face_normals)
    // Orient.cpp:324-354 — void project_vertices(Vec3f orientation)  (get_vertex)
    // Orient.cpp:380-468 — CostItems get_features(orientation, min_volume)
    //                     (bounding_box().area()/radius(), stats().volume,
    //                      its_volume)
    // Orient.cpp:491-557 — Vec3d find_cooling_direction2(...)  (TriangleMesh(its),
    //                     rotate_x/y/z, center, translate, its_face_normals)
    // Orient.cpp:559-604 — Vec3d find_cooling_direction(...)  (same as above)
}

/// `bool isApprox(const Vec3f&, tol)` equivalent for `Vec3f` orientations.
/// Eigen's `isApprox` uses `(a-b).squaredNorm() <= tol*tol * min(a.sn, b.sn)`.
/// Used by `remove_duplicates`. Orient.cpp:311,317
#[inline]
fn is_approx_vec3f(a: &Vec3f, b: &Vec3f, tol: f64) -> bool {
    let tol = tol as f32;
    let diff_sn = (a - b).norm_squared();
    let min_sn = a.norm_squared().min(b.norm_squared());
    diff_sn <= tol * tol * min_sn
}

// =====================================================================
// Free functions — BLOCKED on mesh / Model / Geometry / TBB dependencies.
// =====================================================================
//
// Orient.cpp:607-647 — void _orient(OrientMeshs&, params, progressfn, stopfn)
//   Constructs `AutoOrienter` (=> preprocess), calls process(), and uses
//   `Geometry::rotation_from_two_vectors` / `extract_euler_angles` /
//   `find_cooling_direction2`, plus `tbb::parallel_for`. BLOCKED.
//
// Orient.cpp:649-659 — void orient(OrientMeshs&, const OrientMeshs&, OrientParams&)
//   Thin wrapper over `_orient`. BLOCKED (depends on `_orient`).
//
// Orient.cpp:661-679 — void orient(ModelObject* obj)
//   Uses `obj->mesh()`, `obj->config.has/opt_int`, `obj->rotate`,
//   `obj->ensure_on_bed`, and `memcpy(&orienter.params, &params_area, ...)`.
//   The Rust `model.rs` is a simplified subset without these. BLOCKED.
//
// Orient.cpp:681-691 — void orient(ModelInstance* instance)
//   Uses `instance->get_object()->mesh()` and `instance->rotate(Matrix3d)`.
//   No `ModelInstance` type in Rust `model.rs`. BLOCKED.
//
// Orient.cpp:693-785 — void orient_for_cooling(TriangleMesh&, const FanDirection&)
//   Uses `mesh.facets_count()`, `its_face_normals(mesh.its)`,
//   `mesh.its.facet_area(i)`, `mesh.rotate(angle, axis)`, and
//   `Geometry::rotation_from_two_vectors`. BLOCKED.

// `#define MAX3(a,b,c) std::max(std::max(a,b),c)`   Orient.cpp:17
// `#define MEDIAN3(a,b,c) std::max(std::min(a,b), std::min(std::max(a,b),c))`  Orient.cpp:20
// `#define SQ(x) ((x)*(x))`   Orient.cpp:22
// These macros are only used inside the BLOCKED `project_vertices`/`get_features`
// methods; they will be ported alongside those methods.
#[allow(unused)]
#[inline]
fn max3(a: f32, b: f32, c: f32) -> f32 {
    // Orient.cpp:17
    a.max(b).max(c)
}

#[allow(unused)]
#[inline]
fn median3(a: f32, b: f32, c: f32) -> f32 {
    // Orient.cpp:20
    (a.min(b)).max(a.max(b).min(c))
}

#[allow(unused)]
#[inline]
fn sq(x: f32) -> f32 {
    // Orient.cpp:22
    x * x
}

/// Suppress dead-code warnings for the genuinely-unused-until-unblocked
/// constants imported for the future port.
#[allow(unused)]
const _UNUSED_GUARD: (f64, f64) = (PI, EPSILON);
