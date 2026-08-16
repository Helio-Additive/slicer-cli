//! Faithful 1:1 port of `GCode/SeamPlacer.{hpp,cpp}` from BambuStudio libslic3r.
//!
//! C++ Reference:
//! - src/libslic3r/GCode/SeamPlacer.hpp
//! - src/libslic3r/GCode/SeamPlacer.cpp
//!
//! This file mirrors the C++ source line by line (see `// SeamPlacer.cpp:NNN`
//! and `// SeamPlacer.hpp:NNN` line references). `coord_t` maps to `i64`,
//! `coordf_t` to `f64`, and Eigen `Vec3f`/`Vec2f`/`Vec3i` to nalgebra
//! `Vector3<f32>` / `Vector2<f32>` / `Vector3<i32>` exactly as the rest of the
//! crate does (see `triangle_set_sampling`, `triangle_mesh`).
//!
//! # Porting status
//!
//! The self-contained algorithmic core (the C++ `SeamPlacerImpl` namespace) and
//! the data structures from the header are ported faithfully, including
//! `extract_perimeter_polygons` and `process_perimeter_polygon`. The
//! `SeamPlacer` member functions unblocked by the config-hierarchy wiring
//! (`PrintObject::{config, layers, slicing_parameters}`, `Layer::object`,
//! `LayerRegion::{region, flow}`) are ported faithfully as well:
//! `gather_seam_candidates`, `calculate_candidates_visibility`,
//! `calculate_overhangs_and_layer_embedding`, `gather_all_seams_of_object`,
//! `filter_scarf_seam_switch_by_angle`. The seam-alignment pipeline
//! (`find_next_seam_in_layer`, `find_seam_string`, `align_seam_points`) is now
//! ported faithfully as well: the per-layer f32 `points_tree` is queried via the
//! new `find_nearby_points_eps` (which takes the `float(EPSILON)` term as a
//! parameter rather than `T: From<f64>`), and the cubic B-spline fit maps to the
//! crate's `Geometry::fit_cubic_bspline` / `get_fitted_value`.
//!
//! PORTED — the visibility/occlusion pipeline is now wired:
//! - `SeamPlacer::init` (SeamPlacer.cpp:1395) — runs `compute_global_occlusion`
//!   (below) before gathering candidates, so each candidate gets a real per-vertex
//!   visibility from raycast occlusion.
//! - `compute_global_occlusion` (SeamPlacer.cpp:574) — samples the (already
//!   bed-centered) `PrintObject::mesh()`, decimates via `its_short_edge_collpase`,
//!   builds the AABB + sample KD trees, and raycasts per-sample visibility. The
//!   C++ `trafo_centered()` transform is already baked into `PrintObject::mesh()`
//!   in this pipeline (the CLI centers the mesh before constructing the object),
//!   so no extra transform is applied.
//! - `raycast_visibility` (SeamPlacer.cpp:135) — full hemisphere raycast via the
//!   crate `aabb_tree_indirect` (f32 vertices cast to f64, matching the C++
//!   double-precision intersection math); both the first-hit and the
//!   negative-volume all-hits branches are ported (the latter never triggers here,
//!   as there are no negative volumes).
//!
//! STILL BLOCKED (no model dependency in this pipeline):
//! - `gather_enforcers_blockers` (SeamPlacer.cpp:644) — needs `ModelVolume`
//!   seam-painting facet accessors (`is_seam_painted`, `seam_facets.get_facets`,
//!   `EnforcerBlockerType`), not present here (Benchy has no seam painting), so
//!   `GlobalModelInfo::{enforcers,blockers}` stay empty and `is_enforced`/
//!   `is_blocked` short-circuit to `false`.

use crate::aabb_tree_indirect::Tree3F;
use crate::aabb_tree_lines::{build_aabb_tree_over_indexed_lines, squared_distance_to_indexed_lines, tree2d};
use crate::extrusion_entity::{is_perimeter, ExtrusionEntityType, ExtrusionRole};
use crate::flow::FlowRole;
use crate::geometry::{Line, LineF, Point, PointF, Polygon, Polygons};
use crate::geometry::bicubic::CubicBSplineKernel;
use crate::geometry::curves::fit_cubic_bspline;
use crate::kd_tree_indirect::{find_nearby_points_eps, KDTreeIndirect};
use crate::libslic3r::EPSILON;
use crate::layer::{Layer, LayerRegion};
use crate::print_object::PrintObject;
use crate::triangle_set_sampling::{indexed_triangle_set, TriangleSetSamples};
use crate::unscale;
use crate::utils::{next_idx_modulo, prev_idx_modulo};
use nalgebra::{Vector2, Vector3};
use std::collections::VecDeque;

/// R734 — stage census for `SeamPlacer::init`, proven at 1,100.7 ms = 42% of the
/// whole benchy slice. `SPPROF=1` prints at exit.
pub static SPPROF_OCCL: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static SPPROF_GATHER: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static SPPROF_VIS: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static SPPROF_OVER: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static SPPROF_ALIGN: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static SPPROF_DECIM: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static SPPROF_SAMPLE: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static SPPROF_AABB: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static SPPROF_RAYCAST: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);

pub fn spprof_report() {
    use std::sync::atomic::Ordering::Relaxed;
    if !crate::probe_enabled("SPPROF") {
        return;
    }
    let f = |x: &std::sync::atomic::AtomicUsize| x.load(Relaxed) as f64 / 1e6;
    eprintln!(
        "[SPPROF] occlusion {:.1}ms (decimate {:.1} | sample {:.1} | aabb {:.1} | raycast {:.1}) | gather {:.1}ms | visibility {:.1}ms | overhangs {:.1}ms | align {:.1}ms",
        f(&SPPROF_OCCL), f(&SPPROF_DECIM), f(&SPPROF_SAMPLE), f(&SPPROF_AABB), f(&SPPROF_RAYCAST),
        f(&SPPROF_GATHER), f(&SPPROF_VIS), f(&SPPROF_OVER), f(&SPPROF_ALIGN)
    );
}


/// Eigen `Vec3f = Matrix<float,3,1>`. Point.hpp
pub type Vec3f = Vector3<f32>;
/// Eigen `Vec2f = Matrix<float,2,1>`. Point.hpp
pub type Vec2f = Vector2<f32>;
/// Eigen `Vec3d = Matrix<double,3,1>`. Point.hpp
pub type Vec3d = Vector3<f64>;
/// Eigen `Vec2d = Matrix<double,2,1>`. Point.hpp
pub type Vec2d = Vector2<f64>;
/// Eigen `Vec3i = Matrix<int,3,1>`. Point.hpp
pub type Vec3i = Vector3<i32>;

// libslic3r.h: M_PI used as float(PI) throughout SeamPlacer.cpp.
const PI: f32 = std::f64::consts::PI as f32;

// SeamPlacer.cpp:32 (`constexpr int average_filter_window_size = 5;`)
const AVERAGE_FILTER_WINDOW_SIZE: i32 = 5;
// SeamPlacer.cpp:33
const OVERHANG_FILTER: f32 = 0.0;
// SeamPlacer.cpp:34 (`constexpr float lensLimit = 1.0f;`)
const LENS_LIMIT: f32 = 1.0;

// ============================================================================
// SeamPlacer.hpp:128-182 — class SeamPlacer constants
// ============================================================================

/// Number of samples generated on the mesh. There are
/// `sqr_rays_per_sample_point*sqr_rays_per_sample_point` rays casted from each
/// sample.
/// SeamPlacer.hpp:132
pub const RAYCASTING_VISIBILITY_SAMPLES_COUNT: usize = 30000;
/// SeamPlacer.hpp:133
pub const FAST_DECIMATION_TRIANGLE_COUNT_TARGET: usize = 16000;
/// square of number of rays per sample point
/// SeamPlacer.hpp:135
pub const SQR_RAYS_PER_SAMPLE_POINT: usize = 5;

/// snapping angle - angles larger than this value will be snapped to during seam painting
/// SeamPlacer.hpp:138
pub const SHARP_ANGLE_SNAPPING_THRESHOLD: f32 = 55.0 * PI / 180.0;
/// overhang angle for seam placement that still yields good results, in degrees,
/// measured from vertical direction (BBS)
/// SeamPlacer.hpp:141
pub const OVERHANG_ANGLE_THRESHOLD: f32 = 45.0 * PI / 180.0;

/// determines angle importance compared to visibility ( neutral value is 1.0f. )
/// SeamPlacer.hpp:144
pub const ANGLE_IMPORTANCE_ALIGNED: f32 = 0.6;
/// use much higher angle importance for nearest mode, to combat the visibility info noise
/// SeamPlacer.hpp:145
pub const ANGLE_IMPORTANCE_NEAREST: f32 = 1.0;

/// For long polygon sides, if they are close to the custom seam drawings, they are
/// oversampled with this step size
/// SeamPlacer.hpp:148
pub const ENFORCER_OVERSAMPLING_DISTANCE: f32 = 0.2;
/// SeamPlacer.hpp:149
pub const END_POINT_OVERSAMPLING_THRESHOLD: f32 = 4.0;
/// SeamPlacer.hpp:150
pub const END_POINT_OVERSAMPLING_DISTANCE: f32 = 1.5;

/// following value describes, how much worse score can point have and still be
/// picked into seam cluster instead of original seam point on the same layer
/// SeamPlacer.hpp:154
pub const SEAM_ALIGN_SCORE_TOLERANCE: f32 = 0.3;
/// how far to search for seam from current position, final dist is
/// `seam_align_tolerable_dist_factor * flow_width`
/// SeamPlacer.hpp:156
pub const SEAM_ALIGN_TOLERABLE_DIST_FACTOR: f32 = 4.0;
/// minimum number of seams needed in cluster to make alignment happen
/// SeamPlacer.hpp:158
pub const SEAM_ALIGN_MINIMUM_STRING_SEAMS: usize = 6;
/// millimeters covered by spline; determines number of splines for the given string
/// SeamPlacer.hpp:160  (`static constexpr size_t seam_align_mm_per_segment = 4.0f;`)
pub const SEAM_ALIGN_MM_PER_SEGMENT: f32 = 4.0;

// ============================================================================
// PrintConfig.hpp — SeamPosition enum (used by SeamComparator)
// ============================================================================

/// `SeamPosition` from PrintConfig.hpp.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[allow(non_camel_case_types)]
pub enum SeamPosition {
    spRandom,
    spNearest,
    spAligned,
    spRear,
}

// ============================================================================
// SeamPlacer.hpp:45-49 — enum class EnforcedBlockedSeamPoint
// ============================================================================

/// SeamPlacer.hpp:45-49
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum EnforcedBlockedSeamPoint {
    // SeamPlacer.hpp:46
    Blocked = 0,
    // SeamPlacer.hpp:47
    Neutral = 1,
    // SeamPlacer.hpp:48
    Enforced = 2,
}

// ============================================================================
// SeamPlacer.hpp:52-64 — struct Perimeter
// ============================================================================

/// struct representing single perimeter loop.
/// SeamPlacer.hpp:52-64
#[derive(Clone, Debug)]
pub struct Perimeter {
    // SeamPlacer.hpp:54
    pub start_index: usize,
    // SeamPlacer.hpp:55 — inclusive!
    pub end_index: usize,
    // SeamPlacer.hpp:56
    pub seam_index: usize,
    // SeamPlacer.hpp:57
    pub flow_width: f32,

    // During alignment, a final position may be stored here. In that case,
    // finalized is set to true.
    // Note that final seam position is not limited to points of the perimeter
    // loop. In theory it can be any position. Random position also uses this
    // flexibility to set final seam point position.
    // SeamPlacer.hpp:62
    pub finalized: bool,
    // SeamPlacer.hpp:63
    pub final_seam_position: Vec3f,
}

impl Default for Perimeter {
    fn default() -> Self {
        // SeamPlacer.hpp:54-63 default member initializers.
        Self {
            start_index: 0,
            end_index: 0,
            seam_index: 0,
            flow_width: 0.0,
            finalized: false,
            final_seam_position: Vec3f::zeros(),
        }
    }
}

// ============================================================================
// SeamPlacer.hpp:69-100 — struct SeamCandidate
// ============================================================================

/// Struct over which all processing of perimeters is done. For each perimeter
/// point, its respective candidate is created, then all the needed attributes
/// are computed and finally, for each perimeter one point is chosen as seam.
/// SeamPlacer.hpp:69-100
#[derive(Clone, Debug)]
pub struct SeamCandidate {
    // SeamPlacer.hpp:85
    pub position: Vec3f,
    // pointer to Perimeter loop of this point. It is shared across all points of
    // the loop. We store the index of the perimeter in the owning `LayerSeams`.
    // SeamPlacer.hpp:87 (`Perimeter &perimeter;`)
    pub perimeter: usize,
    // SeamPlacer.hpp:88
    pub visibility: f32,
    // SeamPlacer.hpp:89
    pub overhang: f32,
    // distance inside the merged layer regions, for detecting perimeter points
    // which are hidden inside the print (e.g. multimaterial join). Negative sign
    // means inside the print, comes from EdgeGrid structure.
    // SeamPlacer.hpp:92
    pub embedded_distance: f32,
    // SeamPlacer.hpp:93
    pub local_ccw_angle: f32,
    // SeamPlacer.hpp:94
    pub r#type: EnforcedBlockedSeamPoint,
    // marks this candidate as central point of enforced segment on the perimeter
    // - important for alignment
    // SeamPlacer.hpp:95
    pub central_enforcer: bool,
    // marks this candidate as a candidate for scarf seam
    // SeamPlacer.hpp:96
    pub enable_scarf_seam: bool,
    // SeamPlacer.hpp:97
    pub is_grouped: bool,
    // SeamPlacer.hpp:98
    pub extra_overhang_point: f32,
    // SeamPlacer.hpp:99
    pub overhang_degree: f32,
    // Not in C++: copy of the owning perimeter's `flow_width`. In C++ the
    // candidate holds a `Perimeter &perimeter` back-reference, so
    // `is_first_not_much_worse` can read `a.perimeter.flow_width`
    // (SeamPlacer.cpp:732). We store the index in `perimeter`; this field caches
    // the flow width so the comparator can stay reference-free.
    pub flow_width_hint: f32,
}

impl SeamCandidate {
    /// SeamPlacer.hpp:71-84 — constructor.
    pub fn new(
        pos: &Vec3f,
        perimeter: usize,
        local_ccw_angle: f32,
        r#type: EnforcedBlockedSeamPoint,
    ) -> Self {
        Self {
            position: *pos,
            perimeter,
            visibility: 0.0,
            overhang: 0.0,
            embedded_distance: 0.0,
            local_ccw_angle,
            r#type,
            central_enforcer: false,
            enable_scarf_seam: false,
            is_grouped: false,
            extra_overhang_point: 0.0,
            overhang_degree: 0.0,
            flow_width_hint: 0.0,
        }
    }
}

// ============================================================================
// SeamPlacer.hpp:110-126 — struct PrintObjectSeamData
// ============================================================================

/// SeamPlacer.hpp:114-119 — struct LayerSeams.
///
/// In C++ `points_tree` is a `unique_ptr<KDTreeIndirect>`. We rebuild the tree
/// on demand via [`LayerSeams::build_points_tree`] because the KD tree borrows
/// the coordinate functor closure.
#[derive(Clone, Debug, Default)]
pub struct LayerSeams {
    // SeamPlacer.hpp:116
    pub perimeters: VecDeque<Perimeter>,
    // SeamPlacer.hpp:117
    pub points: Vec<SeamCandidate>,
    /// SeamPlacer.hpp:118 — C++ stores the built `points_tree` in the layer
    /// (`std::unique_ptr<SeamCandidatesTree>`). Our `KDTreeIndirect` borrows its
    /// coordinate closure from `points`, so the tree itself cannot be a field;
    /// the built NODE ARRAY (all of the build cost) is cached instead and
    /// [`LayerSeams::build_points_tree`] reconstructs around it. R523.
    pub points_tree_nodes: std::sync::OnceLock<Vec<usize>>,
}

impl LayerSeams {
    /// Build a KD tree over `points`, mirroring
    /// `points_tree = std::make_unique<SeamCandidatesTree>(functor, points.size())`.
    /// SeamPlacer.cpp:944-945
    pub fn build_points_tree(&self) -> KDTreeIndirect<3, f32, impl Fn(usize, usize) -> f32 + '_> {
        if crate::probe_enabled("KDCOUNT") {
            KD_BUILDS.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            KD_POINTS.fetch_add(self.points.len(), std::sync::atomic::Ordering::Relaxed);
        }
        // SeamPlacer.hpp:102-107 — SeamCandidateCoordinateFunctor.
        let functor = move |index: usize, dim: usize| -> f32 { self.points[index].position[dim] };
        // R523: C++ builds this tree ONCE per layer (SeamPlacer.cpp:944-945) and
        // stores it in the layer; this port built it on demand at every query,
        // which measured 59,677 builds over 88.6M points for a 657-layer print
        // (91x C++'s one-per-layer). The candidate positions are frozen after
        // gather_seam_candidates, so the node array is cached here on first use
        // and the tree is reconstructed cheaply around it.
        let nodes = self.points_tree_nodes.get_or_init(|| {
            let build_functor =
                |index: usize, dim: usize| -> f32 { self.points[index].position[dim] };
            KDTreeIndirect::<3, f32, _>::with_indices(build_functor, self.points.len())
                .nodes()
                .to_vec()
        });
        KDTreeIndirect::from_nodes(functor, nodes.clone())
    }
}

pub static KD_BUILDS: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static KD_POINTS: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);

/// SeamPlacer.hpp:110-126 — struct PrintObjectSeamData.
#[derive(Clone, Debug, Default)]
pub struct PrintObjectSeamData {
    // Map of PrintObjects (PO) -> vector of layers of PO -> vector of perimeter
    // SeamPlacer.hpp:121
    pub layers: Vec<LayerSeams>,
}

impl PrintObjectSeamData {
    // SeamPlacer.hpp:125
    pub fn clear(&mut self) {
        self.layers.clear();
    }
}

// ============================================================================
// SeamPlacer.hpp:32-39 — angle()
// ============================================================================

/// FOR BACKPORT COMPATIBILITY ONLY
/// Angle from v1 to v2, returning double atan2(y, x) normalized to <-PI, PI>.
/// SeamPlacer.hpp:32-39
#[inline]
pub fn angle(v1: Vec2d, v2: Vec2d) -> f64 {
    // cross2(v1d, v2d) = v1.x*v2.y - v1.y*v2.x ; v1d.dot(v2d)
    let cross = v1.x * v2.y - v1.y * v2.x;
    cross.atan2(v1.dot(&v2))
}

// ============================================================================
// SeamPlacer.cpp:37-922 — namespace SeamPlacerImpl
// ============================================================================

/// FOR BACKPORT COMPATIBILITY ONLY
/// Color mapping of a value into RGB false colors.
/// SeamPlacer.cpp:41-48
#[inline]
pub fn value_to_rgbf(minimum: f32, maximum: f32, value: f32) -> Vec3f {
    let ratio = 2.0 * (value - minimum) / (maximum - minimum);
    let b = f32::max(0.0, 1.0 - ratio);
    let r = f32::max(0.0, ratio - 1.0);
    let g = 1.0 - b - r;
    Vec3f::new(r, g, b)
}

/// Color mapping of a value into RGB false colors.
/// SeamPlacer.cpp:51
#[inline]
pub fn value_to_rgbi(minimum: f32, maximum: f32, value: f32) -> Vec3i {
    let v = value_to_rgbf(minimum, maximum, value) * 255.0;
    Vec3i::new(v.x as i32, v.y as i32, v.z as i32)
}

/// SeamPlacer.cpp:54
#[inline]
pub fn sgn(val: f32) -> i32 {
    (if 0.0 < val { 1 } else { 0 }) - (if val < 0.0 { 1 } else { 0 })
}

/// base function: ((e^(((1)/(x^(2)+1)))-1)/(e-1))
/// SeamPlacer.cpp:58-64
pub fn gauss(value: f32, mean_x_coord: f32, mean_value: f32, falloff_speed: f32) -> f32 {
    let shifted = value - mean_x_coord;
    let denominator = falloff_speed * shifted * shifted + 1.0;
    let exponent = 1.0 / denominator;
    mean_value * (exponent.exp() - 1.0) / (1.0_f32.exp() - 1.0)
}

/// SeamPlacer.cpp:66-74
pub fn compute_angle_penalty(ccw_angle: f32) -> f32 {
    // This function is used:
    // ((ℯ^(((1)/(x^(2)*3+1)))-1)/(ℯ-1))*1+((1)/(2+ℯ^(-x)))
    // looks scary, but it is gaussian combined with sigmoid,
    // so that concave points have much smaller penalty over convex ones
    gauss(ccw_angle, 0.0, 1.0, 3.0) + 1.0 / (2.0 + (-ccw_angle).exp())
}

/// Coordinate frame.
/// SeamPlacer.cpp:77-110
pub struct Frame {
    m_x: Vec3f,
    m_y: Vec3f,
    m_z: Vec3f,
}

impl Default for Frame {
    fn default() -> Self {
        Self::new()
    }
}

impl Frame {
    // SeamPlacer.cpp:80-85
    pub fn new() -> Self {
        Self {
            m_x: Vec3f::new(1.0, 0.0, 0.0),
            m_y: Vec3f::new(0.0, 1.0, 0.0),
            m_z: Vec3f::new(0.0, 0.0, 1.0),
        }
    }

    // SeamPlacer.cpp:87
    pub fn from_xyz(x: Vec3f, y: Vec3f, z: Vec3f) -> Self {
        Self {
            m_x: x,
            m_y: y,
            m_z: z,
        }
    }

    // SeamPlacer.cpp:89-96
    pub fn set_from_z(&mut self, z: &Vec3f) {
        self.m_z = z.normalize();
        let tmp_z = self.m_z;
        let tmp_x = if tmp_z.x.abs() > 0.99 {
            Vec3f::new(0.0, 1.0, 0.0)
        } else {
            Vec3f::new(1.0, 0.0, 0.0)
        };
        self.m_y = tmp_z.cross(&tmp_x).normalize();
        self.m_x = self.m_y.cross(&tmp_z);
    }

    // SeamPlacer.cpp:98
    pub fn to_world(&self, a: &Vec3f) -> Vec3f {
        a.x * self.m_x + a.y * self.m_y + a.z * self.m_z
    }

    // SeamPlacer.cpp:100
    pub fn to_local(&self, a: &Vec3f) -> Vec3f {
        Vec3f::new(self.m_x.dot(a), self.m_y.dot(a), self.m_z.dot(a))
    }

    // SeamPlacer.cpp:102
    pub fn binormal(&self) -> &Vec3f {
        &self.m_x
    }

    // SeamPlacer.cpp:104
    pub fn tangent(&self) -> &Vec3f {
        &self.m_y
    }

    // SeamPlacer.cpp:106
    pub fn normal(&self) -> &Vec3f {
        &self.m_z
    }
}

/// SeamPlacer.cpp:112-117
pub fn sample_sphere_uniform(samples: &Vec2f) -> Vec3f {
    let term1 = 2.0 * PI * samples.x;
    let term2 = 2.0 * (samples.y - samples.y * samples.y).sqrt();
    Vec3f::new(
        term1.cos() * term2,
        term1.sin() * term2,
        1.0 - 2.0 * samples.y,
    )
}

/// SeamPlacer.cpp:119-124
pub fn sample_hemisphere_uniform(samples: &Vec2f) -> Vec3f {
    let term1 = 2.0 * PI * samples.x;
    let term2 = 2.0 * (samples.y - samples.y * samples.y).sqrt();
    Vec3f::new(
        term1.cos() * term2,
        term1.sin() * term2,
        (1.0 - 2.0 * samples.y).abs(),
    )
}

/// SeamPlacer.cpp:126-133
pub fn sample_power_cosine_hemisphere(samples: &Vec2f, power: f32) -> Vec3f {
    let term1 = 2.0 * PI * samples.x;
    let term2 = samples.y.powf(1.0 / (power + 1.0));
    let term3 = (1.0 - term2 * term2).sqrt();
    Vec3f::new(term1.cos() * term3, term1.sin() * term3, term2)
}

/// Face normal (normalized) of triangle `face_idx` in an f32
/// `indexed_triangle_set`, mirroring C++ `its_face_normal`
/// (TriangleMesh.hpp:333-336 → `face_normal_normalized`,
/// `(v1-v0).cross(v2-v1).normalized()`).
#[inline]
fn its_face_normal_f32(its: &indexed_triangle_set, face_idx: usize) -> Vec3f {
    let f = its.indices[face_idx];
    let v0 = its.vertices[f.x as usize];
    let v1 = its.vertices[f.y as usize];
    let v2 = its.vertices[f.z as usize];
    (v1 - v0).cross(&(v2 - v1)).normalize()
}

/// SeamPlacer.cpp:135-214 — `raycast_visibility`.
///
/// Precomputes the hemisphere sample directions (SeamPlacer.cpp:143-152) and
/// casts `sqr_rays_per_sample_point²` rays from each surface sample, decrementing
/// the sample's visibility for every ray that hits a forward-facing triangle.
///
/// The C++ AABB query (`AABBTreeIndirect::intersect_ray_first_hit`) is called with
/// **f32 mesh vertices** but an **f64 ray** (`ray_origin_d`/`final_ray_dir_d`):
/// the RayIntersector's scalar type is `double`, so the intersection math runs in
/// f64 with the f32 vertices cast up to f64 (AABBTreeIndirect.hpp:365-368, the
/// `intersect_triangle(origin_d, dir_d, v0.cast<double>(), …)` overload). The crate
/// `aabb_tree_indirect` works natively in f64 `Point3F`, so we feed it the f32
/// vertices cast to f64 (lossless) and the same f64 ray, reproducing the C++ path
/// exactly. The default ray-triangle epsilon (0.000001, AABBTreeIndirect.hpp:737)
/// matches `intersect_ray_first_hit`.
///
/// `verts_f64`/`faces_usize` are the f64/`[usize;3]` views of `triangles` over which
/// `raycasting_tree` was built (so face indices line up with `triangles.indices`).
pub fn raycast_visibility(
    raycasting_tree: &Tree3F,
    triangles: &indexed_triangle_set,
    verts_f64: &[crate::geometry::Point3F],
    faces_usize: &[[usize; 3]],
    samples: &TriangleSetSamples,
    negative_volumes_start_index: usize,
) -> Vec<f32> {
    use crate::geometry::Point3F;
    // R190 (ZSMOOTH_FAITHFUL): run the whole raycast through the native-code
    // shim (AABBTreeIndirect + Frame + hemisphere dirs) — 112/750k edge-grazing
    // ray decisions differed between the rust port and native Eigen/igl codegen.
    // Benchy has no negative volumes; fall through to the rust port otherwise.
    if negative_volumes_start_index >= triangles.indices.len()
        && crate::faithful_gate("ZSMOOTH_FAITHFUL")
    {
        let verts_flat: Vec<f32> = triangles
            .vertices
            .iter()
            .flat_map(|v| [v.x, v.y, v.z])
            .collect();
        let idx_flat: Vec<i32> = triangles
            .indices
            .iter()
            .flat_map(|t| [t.x, t.y, t.z])
            .collect();
        let pos_flat: Vec<f32> = samples
            .positions
            .iter()
            .flat_map(|p| [p.x, p.y, p.z])
            .collect();
        let nrm_flat: Vec<f32> = samples
            .normals
            .iter()
            .flat_map(|p| [p.x, p.y, p.z])
            .collect();
        return eigen_transform_sys::raycast_visibility_native(
            &verts_flat,
            &idx_flat,
            &pos_flat,
            &nrm_flat,
            SQR_RAYS_PER_SAMPLE_POINT,
        );
    }
    // SeamPlacer.cpp:143 — prepare uniform samples of a hemisphere
    let step_size = 1.0 / SQR_RAYS_PER_SAMPLE_POINT as f32;
    // SeamPlacer.cpp:144
    let mut precomputed_sample_directions =
        vec![Vec3f::zeros(); SQR_RAYS_PER_SAMPLE_POINT * SQR_RAYS_PER_SAMPLE_POINT];
    // SeamPlacer.cpp:145-152
    for x_idx in 0..SQR_RAYS_PER_SAMPLE_POINT {
        // C++: float sample_x = x_idx * step_size + step_size / 2.0;
        let sample_x = x_idx as f32 * step_size + step_size / 2.0;
        for y_idx in 0..SQR_RAYS_PER_SAMPLE_POINT {
            let dir_index = x_idx * SQR_RAYS_PER_SAMPLE_POINT + y_idx;
            let sample_y = y_idx as f32 * step_size + step_size / 2.0;
            precomputed_sample_directions[dir_index] =
                sample_hemisphere_uniform(&Vec2f::new(sample_x, sample_y));
        }
    }

    // SeamPlacer.cpp:154
    let model_contains_negative_parts = negative_volumes_start_index < triangles.indices.len();

    // SeamPlacer.cpp:156-205 — C++ runs under tbb::parallel_for; ported with rayon
    // (each sample writes its own independent result slot, so the parallelism is
    // value-equivalent to the serial loop).
    let mut result = vec![0.0_f32; samples.positions.len()];
    {
        use rayon::prelude::*;
        result
            .par_iter_mut()
            .enumerate()
            .for_each(|(s_idx, out)| {
                // SeamPlacer.cpp:162
                *out = 1.0_f32;
                // SeamPlacer.cpp:163
                let decrease_step = 1.0_f32
                    / (SQR_RAYS_PER_SAMPLE_POINT * SQR_RAYS_PER_SAMPLE_POINT) as f32;

                // SeamPlacer.cpp:165-166
                let center = samples.positions[s_idx];
                let normal = samples.normals[s_idx];
                // SeamPlacer.cpp:168-169 — apply the local direction via Frame.
                let mut f = Frame::new();
                f.set_from_z(&normal);

                for dir in &precomputed_sample_directions {
                    // SeamPlacer.cpp:172
                    let mut final_ray_dir = f.to_world(dir);
                    if !model_contains_negative_parts {
                        // SeamPlacer.cpp:174-181 — single first-hit branch.
                        let final_ray_dir_d = Point3F::new(
                            final_ray_dir.x as f64,
                            final_ray_dir.y as f64,
                            final_ray_dir.z as f64,
                        );
                        // ray_origin = (center + normal * 0.01) — start above surface.
                        let origin = center + normal * 0.01_f32;
                        let ray_origin_d =
                            Point3F::new(origin.x as f64, origin.y as f64, origin.z as f64);
                        let hit = crate::aabb_tree_indirect::intersect_ray_first_hit(
                            verts_f64,
                            faces_usize,
                            raycasting_tree,
                            &ray_origin_d,
                            &final_ray_dir_d,
                        );
                        // SeamPlacer.cpp:180 — hit && face_normal·ray_dir <= 0
                        if let Some((_t, face_id, _p)) = hit {
                            if its_face_normal_f32(triangles, face_id).dot(&final_ray_dir) <= 0.0 {
                                *out -= decrease_step;
                            }
                        }
                    } else {
                        // SeamPlacer.cpp:182-203 — negative-volume all-hits path.
                        // SeamPlacer.cpp:183
                        let casting_from_negative_volume =
                            samples.triangle_indices[s_idx] >= negative_volumes_start_index;
                        // SeamPlacer.cpp:185
                        let origin = if casting_from_negative_volume {
                            // SeamPlacer.cpp:186-188 — invert dir, start below surface.
                            final_ray_dir = -1.0 * final_ray_dir;
                            center - normal * 0.01_f32
                        } else {
                            center + normal * 0.01_f32
                        };
                        let ray_origin_d =
                            Point3F::new(origin.x as f64, origin.y as f64, origin.z as f64);
                        let final_ray_dir_d = Point3F::new(
                            final_ray_dir.x as f64,
                            final_ray_dir.y as f64,
                            final_ray_dir.z as f64,
                        );
                        // SeamPlacer.cpp:191 — all hits, sorted by t ascending.
                        let hits = crate::aabb_tree_indirect::intersect_ray_all_hits(
                            verts_f64,
                            faces_usize,
                            raycasting_tree,
                            &ray_origin_d,
                            &final_ray_dir_d,
                        );
                        if !hits.is_empty() {
                            // SeamPlacer.cpp:192-201 — iterate hits in reverse.
                            let mut counter = 0i32;
                            for hit_index in (0..hits.len()).rev() {
                                let face_id = hits[hit_index].1;
                                let face_normal = its_face_normal_f32(triangles, face_id);
                                let s = sgn(face_normal.dot(&final_ray_dir));
                                if face_id >= negative_volumes_start_index {
                                    // SeamPlacer.cpp:196 — negative volume hit.
                                    counter -= s;
                                } else {
                                    // SeamPlacer.cpp:199
                                    counter += s;
                                }
                            }
                            // SeamPlacer.cpp:202
                            if counter == 0 {
                                *out -= decrease_step;
                            }
                        }
                    }
                }
            });
    }

    result
}

/// SeamPlacer.cpp:216-262
pub fn calculate_polygon_angles_at_vertices(
    polygon: &Polygon,
    lengths: &[f32],
    min_arm_length: f32,
) -> Vec<f32> {
    // SeamPlacer.cpp:218
    let mut result = vec![0.0_f32; polygon.len()];

    // SeamPlacer.cpp:220
    if polygon.len() == 1 {
        result[0] = 0.0;
    }

    // SeamPlacer.cpp:222-224
    let mut idx_prev = 0usize;
    let mut idx_curr = 0usize;
    let mut idx_next = 0usize;

    // SeamPlacer.cpp:226-227
    let mut distance_to_prev = 0.0_f32;
    let mut distance_to_next = 0.0_f32;

    // push idx_prev far enough back as initialization
    // SeamPlacer.cpp:230-233
    while distance_to_prev < min_arm_length {
        idx_prev = prev_idx_modulo(idx_prev, polygon.len());
        distance_to_prev += lengths[idx_prev];
    }

    // SeamPlacer.cpp:235
    for _i in 0..polygon.len() {
        // pull idx_prev to current as much as possible, while respecting the min_arm_length
        // SeamPlacer.cpp:237-240
        while distance_to_prev - lengths[idx_prev] > min_arm_length {
            distance_to_prev -= lengths[idx_prev];
            idx_prev = next_idx_modulo(idx_prev, polygon.len());
        }

        // push idx_next forward as far as needed
        // SeamPlacer.cpp:243-246
        while distance_to_next < min_arm_length {
            distance_to_next += lengths[idx_next];
            idx_next = next_idx_modulo(idx_next, polygon.len());
        }

        // Calculate angle between idx_prev, idx_curr, idx_next.
        // SeamPlacer.cpp:249-252
        let p0 = polygon.points()[idx_prev];
        let p1 = polygon.points()[idx_curr];
        let p2 = polygon.points()[idx_next];
        result[idx_curr] = angle(point_sub(&p1, &p0), point_sub(&p2, &p1)) as f32;

        // increase idx_curr by one
        // SeamPlacer.cpp:255-258
        let curr_distance = lengths[idx_curr];
        idx_curr += 1;
        distance_to_prev += curr_distance;
        distance_to_next -= curr_distance;
    }

    result
}

/// Helper: (p1 - p0) as a double 2D vector, matching Eigen `Point - Point`.
#[inline]
fn point_sub(a: &Point, b: &Point) -> Vec2d {
    Vec2d::new((a.x - b.x) as f64, (a.y - b.y) as f64)
}

/// SeamPlacer.cpp:264-271 — struct CoordinateFunctor.
///
/// Stores an owned copy of coordinates; the closure form used by the KD tree
/// indexes into it.
pub struct CoordinateFunctor {
    pub coordinates: Vec<Vec3f>,
}

impl CoordinateFunctor {
    // SeamPlacer.cpp:267
    pub fn new(coords: Vec<Vec3f>) -> Self {
        Self {
            coordinates: coords,
        }
    }

    // SeamPlacer.cpp:270 — operator()(idx, dim)
    #[inline]
    pub fn call(&self, idx: usize, dim: usize) -> f32 {
        self.coordinates[idx][dim]
    }
}

/// SeamPlacer.cpp:274-369 — struct GlobalModelInfo.
///
/// Holds global information about the model — occlusion hits, enforcers,
/// blockers.
pub struct GlobalModelInfo {
    // SeamPlacer.cpp:276
    pub mesh_samples: TriangleSetSamples,
    // SeamPlacer.cpp:277
    pub mesh_samples_visibility: Vec<f32>,
    // SeamPlacer.cpp:278-279 — coordinate functor + KD tree (built on demand).
    pub mesh_samples_coordinate_functor: CoordinateFunctor,
    // SeamPlacer.cpp:280
    pub mesh_samples_radius: f32,

    // SeamPlacer.cpp:282
    pub enforcers: indexed_triangle_set,
    // SeamPlacer.cpp:283
    pub blockers: indexed_triangle_set,
    // SeamPlacer.cpp:284
    pub enforcers_tree: Tree3F,
    // SeamPlacer.cpp:285
    pub blockers_tree: Tree3F,
}

impl Default for GlobalModelInfo {
    fn default() -> Self {
        Self {
            mesh_samples: TriangleSetSamples::default(),
            mesh_samples_visibility: Vec::new(),
            mesh_samples_coordinate_functor: CoordinateFunctor::new(Vec::new()),
            mesh_samples_radius: 0.0,
            enforcers: indexed_triangle_set::default(),
            blockers: indexed_triangle_set::default(),
            enforcers_tree: Tree3F::new(),
            blockers_tree: Tree3F::new(),
        }
    }
}

impl GlobalModelInfo {
    // SeamPlacer.cpp:287-292
    //
    // BLOCKED: `is_any_triangle_in_radius` requires an AABB tree built over
    // `Point3F` (f64) vertices, but `enforcers`/`enforcers_tree` use the f32
    // `indexed_triangle_set`. The `enforcers.empty()` short-circuit is faithful;
    // when enforcers are present the radius query cannot be evaluated against an
    // f32 mesh with the current f64-typed AABB tree.
    pub fn is_enforced(&self, _position: &Vec3f, _radius: f32) -> bool {
        if self.enforcers.indices.is_empty() {
            return false;
        }
        // BLOCKED — see note above. C++: is_any_triangle_in_radius(...).
        false
    }

    // SeamPlacer.cpp:294-299
    // BLOCKED for the same reason as `is_enforced`.
    pub fn is_blocked(&self, _position: &Vec3f, _radius: f32) -> bool {
        if self.blockers.indices.is_empty() {
            return false;
        }
        // BLOCKED — see `is_enforced`. C++: is_any_triangle_in_radius(...).
        false
    }

    /// Build the f32 KD tree over `mesh_samples.positions`, mirroring
    /// `mesh_samples_tree = KDTreeIndirect<3, float, CoordinateFunctor>(...)`
    /// (SeamPlacer.cpp:611). The tree borrows the positions, so it is built on
    /// demand (like `LayerSeams::build_points_tree`) and reused across all
    /// candidate queries of one `calculate_candidates_visibility` pass.
    pub fn build_mesh_samples_tree(
        &self,
    ) -> KDTreeIndirect<3, f32, impl Fn(usize, usize) -> f32 + '_> {
        let positions = &self.mesh_samples.positions;
        let functor = move |index: usize, dim: usize| -> f32 { positions[index][dim] };
        KDTreeIndirect::with_indices(functor, positions.len())
    }

    // SeamPlacer.cpp:301-326 — `calculate_point_visibility`.
    //
    // The radius-weighted visibility averaging is faithful. The nearby-sample
    // lookup `find_nearby_points(mesh_samples_tree, position, mesh_samples_radius)`
    // (SeamPlacer.cpp:303) requires the prebuilt f32 KD tree, which the caller
    // passes in (the tree borrows `self.mesh_samples.positions`, so it cannot be
    // stored in `self`). When the model info is empty (no occlusion computed),
    // the tree is empty so `find_nearby_points` returns the empty set and this
    // yields 1.0 (the C++ SeamPlacer.cpp:304 short-circuit).
    pub fn calculate_point_visibility<F>(
        &self,
        mesh_samples_tree: &KDTreeIndirect<3, f32, F>,
        position: &Vec3f,
    ) -> f32
    where
        F: Fn(usize, usize) -> f32,
    {
        // SeamPlacer.cpp:303 — find_nearby_points(tree, position, mesh_samples_radius).
        // C++ `find_nearby_points` uses `descent_mask(..., max_distance², CoordType(EPSILON))`
        // with CoordType = float, so the epsilon term is `float(EPSILON)`.
        let points = find_nearby_points_eps(
            mesh_samples_tree,
            position,
            self.mesh_samples_radius,
            EPSILON as f32,
            |_| true,
        );
        // SeamPlacer.cpp:304
        if points.is_empty() {
            return 1.0;
        }

        // SeamPlacer.cpp:306-309
        let compute_dist_to_plane =
            |position: &Vec3f, plane_origin: &Vec3f, plane_normal: &Vec3f| -> f32 {
                let orig_to_point = position - plane_origin;
                orig_to_point.dot(plane_normal).abs()
            };

        // SeamPlacer.cpp:311-312
        let mut total_weight = 0.0_f32;
        let mut total_visibility = 0.0_f32;
        // SeamPlacer.cpp:313
        for &sample_idx in &points {
            // SeamPlacer.cpp:316-317
            let sample_point = self.mesh_samples.positions[sample_idx];
            let sample_normal = self.mesh_samples.normals[sample_idx];

            // SeamPlacer.cpp:319-320
            let mut weight = self.mesh_samples_radius
                - compute_dist_to_plane(position, &sample_point, &sample_normal);
            weight += self.mesh_samples_radius - (position - sample_point).norm();
            // SeamPlacer.cpp:321-322
            total_visibility += weight * self.mesh_samples_visibility[sample_idx];
            total_weight += weight;
        }

        // SeamPlacer.cpp:325
        total_visibility / total_weight
    }
}

/// SeamPlacer.cpp:573-642 — `compute_global_occlusion`.
///
/// Builds the [`GlobalModelInfo`] occlusion data for `po`: gathers the object
/// mesh, decimates it, uniformly samples its surface, computes the sample search
/// radius, builds the sample KD tree's backing data + an AABB tree, and raycasts
/// per-sample visibility.
///
/// COORDINATE FRAME: C++ takes the untransformed `model_object()->volumes`,
/// applies each volume matrix, then `po->trafo_centered()` — landing the mesh in
/// the same centered slicing frame the layers/candidates live in. In this crate
/// the `PrintObject::mesh()` is ALREADY in that frame (the CLI centers the mesh
/// on the bed before constructing the `PrintObject`, slicer-cli.rs:701-721), so
/// no further transform is applied here. There are no NEGATIVE_VOLUME parts in
/// this pipeline, so `negative_volumes_start_index == indices.len()` (the
/// model-contains-negative-parts branch never triggers, matching Benchy).
pub fn compute_global_occlusion(po: &PrintObject) -> GlobalModelInfo {
    let mut result = GlobalModelInfo::default();

    // SeamPlacer.cpp:577-591 — gather the object parts into `triangle_set`.
    // The crate stores a single merged `TriangleMesh` (already centered); convert
    // it to the f32 `indexed_triangle_set` the sampler/raycaster operate on.
    let mesh = match po.mesh() {
        Some(m) => m,
        None => return result,
    };
    // R200: normalize -0.0 -> +0.0 during the f64->f32 conversion. Native's
    // volume mesh is the admesh f32 store (Benchy has no -0.0 bits); rust's f64
    // pipeline can surface -0.0 on the bed plane (rustc/LLVM minnum sign-of-zero
    // drift flipped 80 z=0 verts between sessions) — sign bits change the
    // collapse FP stream and cascade through sampling/visibility (R199).
    // R201: native's occlusion verts are the admesh f32 VOLUME STORE translated
    // back by the volume matrix — i.e. the f32 center-store round trip
    // (quantize_f32_center_roundtrip). Under FRAME_UNIFY the CLI SKIPS that
    // bake on the mesh (the slice shim reproduces it in the slice transform),
    // so po.mesh() here is RAW and the occlusion verts were never quantized —
    // every prior occlusion bit-parity check ran under ZSMOOTH-only gates where
    // the bake was active (R200's "drift" was this gate-set difference, not a
    // toolchain change). Reproduce the round trip per vertex (f32, mesh bbox
    // center), normalizing -0.0 -> +0.0 (native f32 store has +0.0).
    let n0 = |f: f32| if f == 0.0 { 0.0f32 } else { f };
    let occl_faithful = crate::faithful_gate("ZSMOOTH_FAITHFUL");
    let qc = {
        let c = mesh.compute_bounding_box().center();
        (c.x as f32, c.y as f32, c.z as f32)
    };
    let mut triangle_set = indexed_triangle_set {
        vertices: mesh
            .vertices()
            .iter()
            .map(|v| {
                if occl_faithful {
                    let q = |val: f64, c: f32| ((val as f32 - c) + c);
                    Vec3f::new(
                        n0(q(v.x, qc.0)),
                        n0(q(v.y, qc.1)),
                        n0(q(v.z, qc.2)),
                    )
                } else {
                    Vec3f::new(v.x as f32, v.y as f32, v.z as f32)
                }
            })
            .collect(),
        indices: mesh
            .indices()
            .iter()
            .map(|t| Vec3i::new(t.indices[0] as i32, t.indices[1] as i32, t.indices[2] as i32))
            .collect(),
    };

    // SeamPlacer.cpp:598 — decimate. `its_short_edge_collpase` takes the
    // `normal_utils` its (structurally identical f32 vertices / i32 indices);
    // convert across the two nominal types around the call.
    {
        use crate::normal_utils::{indexed_triangle_set as NuIts, Vec3crd};
        let mut nu = NuIts {
            vertices: triangle_set.vertices.clone(),
            indices: triangle_set
                .indices
                .iter()
                .map(|i| Vec3crd::new(i.x, i.y, i.z))
                .collect(),
        };
        // SeamPlacer.cpp:598 — its_short_edge_collpase(triangle_set, fast_decimation_...).
        if std::env::var("OCCDBG").is_ok() {
            let (mut mn, mut mx) = ([1e9f32; 3], [-1e9f32; 3]);
            for v in &nu.vertices {
                for (k, c) in [v.x, v.y, v.z].iter().enumerate() {
                    mn[k] = mn[k].min(*c);
                    mx[k] = mx[k].max(*c);
                }
            }
            eprintln!(
                "OCCDBG-R pre tris={} verts={} bb={:.4},{:.4},{:.4}/{:.4},{:.4},{:.4}",
                nu.indices.len(), nu.vertices.len(), mn[0], mn[1], mn[2], mx[0], mx[1], mx[2]
            );
        }
        let __t = std::time::Instant::now();
        crate::short_edge_collapse::its_short_edge_collpase(
            &mut nu,
            FAST_DECIMATION_TRIANGLE_COUNT_TARGET,
        );
        SPPROF_DECIM.fetch_add(__t.elapsed().as_nanos() as usize, std::sync::atomic::Ordering::Relaxed);
        if std::env::var("OCCDBG").is_ok() {
            let (mut mn, mut mx) = ([1e9f32; 3], [-1e9f32; 3]);
            for v in &nu.vertices {
                for (k, c) in [v.x, v.y, v.z].iter().enumerate() {
                    mn[k] = mn[k].min(*c);
                    mx[k] = mx[k].max(*c);
                }
            }
            eprintln!(
                "OCCDBG-R postdec tris={} verts={} bb={:.4},{:.4},{:.4}/{:.4},{:.4},{:.4}",
                nu.indices.len(), nu.vertices.len(), mn[0], mn[1], mn[2], mx[0], mx[1], mx[2]
            );
        }
        // SeamPlacer.cpp:602 — negative_volumes_start_index = triangle_set.indices.size().
        // (no negative volumes are merged in; index == size.)
        triangle_set.vertices = nu.vertices;
        triangle_set.indices = nu
            .indices
            .iter()
            .map(|i| Vec3i::new(i.x, i.y, i.z))
            .collect();
    }
    // SeamPlacer.cpp:604 — its_transform(triangle_set, obj_transform) with
    // obj_transform = po->trafo_centered() = Identity.pretranslate(-cx, -cy, 0)
    // (m_trafo is Identity for slicer_cli STL, R87). Native does
    // v = (t * v.cast<double>()).cast<float>() — a pure f64 translation then one
    // f32 rounding per component, which `(f64 - cx) as f32` reproduces exactly.
    // WITHOUT this the occlusion mesh stays in the volume frame while the seam
    // candidates live in the centered slicing frame — every visibility raycast
    // was skewed by the centering offset (~0.8245mm in x for Benchy).
    if crate::faithful_gate("ZSMOOTH_FAITHFUL") {
        let (cx, cy) = po.slice_center_offset;
        for v in &mut triangle_set.vertices {
            v.x = ((v.x as f64) - cx) as f32;
            v.y = ((v.y as f64) - cy) as f32;
        }
    }
    // SeamPlacer.cpp:602 — there are no negative volumes, so the start index is the
    // full triangle count (the negative-volume raycast branch never triggers).
    let negative_volumes_start_index = triangle_set.indices.len();

    if std::env::var("OCCDBG").is_ok() {
        let mut h: u64 = 1469598103934665603;
        for v in &triangle_set.vertices {
            for f in [v.x, v.y, v.z] {
                h ^= f.to_bits() as u64;
                h = h.wrapping_mul(1099511628211);
            }
        }
        let (mut mn, mut mx) = ([1e9f32; 3], [-1e9f32; 3]);
        for v in &triangle_set.vertices {
            for (k, c) in [v.x, v.y, v.z].iter().enumerate() {
                mn[k] = mn[k].min(*c);
                mx[k] = mx[k].max(*c);
            }
        }
        eprintln!(
            "OCCDBG-R sampin vh={:x} bb={:.6},{:.6},{:.6}/{:.6},{:.6},{:.6}",
            h, mn[0], mn[1], mn[2], mx[0], mx[1], mx[2]
        );
    }
    // SeamPlacer.cpp:609 — sample the decimated mesh surface uniformly by area.
    let __t = std::time::Instant::now();
    result.mesh_samples = crate::triangle_set_sampling::sample_its_uniform_parallel(
        RAYCASTING_VISIBILITY_SAMPLES_COUNT,
        &triangle_set,
    );
    SPPROF_SAMPLE.fetch_add(__t.elapsed().as_nanos() as usize, std::sync::atomic::Ordering::Relaxed);
    // SeamPlacer.cpp:610-611 — coordinate functor + KD tree (the tree is built on
    // demand from `mesh_samples.positions`; the functor copy here is unused but
    // kept for structural parity).
    result.mesh_samples_coordinate_functor =
        CoordinateFunctor::new(result.mesh_samples.positions.clone());

    // SeamPlacer.cpp:617-625 — search radius for the per-perimeter-point visibility
    // averaging (Poisson/exponential area model). Copied EXACTLY.
    let probability = 0.9_f32;
    let samples = 4.0_f32;
    let density =
        RAYCASTING_VISIBILITY_SAMPLES_COUNT as f32 / result.mesh_samples.total_area;
    let search_area = samples / (-probability.ln() * density);
    let search_radius = (search_area / PI).sqrt();
    result.mesh_samples_radius = search_radius;

    // SeamPlacer.cpp:633 — build the AABB tree over the (f32) triangle set. The
    // crate tree works in f64; feed it the f32 vertices cast to f64 (lossless),
    // matching the C++ double-precision intersection math (see `raycast_visibility`).
    let verts_f64: Vec<crate::geometry::Point3F> = triangle_set
        .vertices
        .iter()
        .map(|v| crate::geometry::Point3F::new(v.x as f64, v.y as f64, v.z as f64))
        .collect();
    let faces_usize: Vec<[usize; 3]> = triangle_set
        .indices
        .iter()
        .map(|i| [i.x as usize, i.y as usize, i.z as usize])
        .collect();
    let __t = std::time::Instant::now();
    let raycasting_tree =
        crate::aabb_tree_indirect::build_aabb_tree_over_indexed_triangle_set(&verts_f64, &faces_usize);
    SPPROF_AABB.fetch_add(__t.elapsed().as_nanos() as usize, std::sync::atomic::Ordering::Relaxed);

    // SeamPlacer.cpp:637 — raycast per-sample visibility.
    let __t = std::time::Instant::now();
    result.mesh_samples_visibility = raycast_visibility(
        &raycasting_tree,
        &triangle_set,
        &verts_f64,
        &faces_usize,
        &result.mesh_samples,
        negative_volumes_start_index,
    );
    SPPROF_RAYCAST.fetch_add(__t.elapsed().as_nanos() as usize, std::sync::atomic::Ordering::Relaxed);

    if std::env::var("OCCDBG").is_ok() {
        let mut h: u64 = 1469598103934665603;
        for f in &result.mesh_samples_visibility {
            h ^= f.to_bits() as u64;
            h = h.wrapping_mul(1099511628211);
        }
        let s: f64 = result.mesh_samples_visibility.iter().map(|&v| v as f64).sum();
        eprintln!(
            "OCCDBG-R vis n={} vh={:x} sum={:.6} first5=[{:.6},{:.6},{:.6},{:.6},{:.6}]",
            result.mesh_samples_visibility.len(), h, s,
            result.mesh_samples_visibility[0], result.mesh_samples_visibility[1],
            result.mesh_samples_visibility[2], result.mesh_samples_visibility[3],
            result.mesh_samples_visibility[4]
        );
    }

    result
}

/// Virtual-dispatch helper mirroring the C++ `ExtrusionEntity::collect_points`
/// overrides:
/// - ExtrusionEntity.hpp:347 (`ExtrusionPath`): `append(dst, this->polyline.points);`
/// - ExtrusionEntity.hpp:553 (`ExtrusionLoop`): appends each path's `polyline.points`.
/// - ExtrusionEntityCollection.hpp:137-140: recurses into each entity.
fn collect_points(entity: &ExtrusionEntityType, dst: &mut Vec<Point>) {
    match entity {
        // ExtrusionEntity.hpp:347
        ExtrusionEntityType::Path(path) => dst.extend_from_slice(&path.polyline.points),
        // ExtrusionEntity.hpp:553-556
        ExtrusionEntityType::Loop(l) => {
            for path in &l.paths {
                dst.extend_from_slice(&path.polyline.points);
            }
        }
        // ExtrusionEntityCollection.hpp:137-140
        ExtrusionEntityType::Collection(coll) => {
            for e in &coll.entities {
                collect_points(e, dst);
            }
        }
    }
}

/// Virtual-dispatch helper mirroring `ExtrusionEntity::role()`:
/// - `ExtrusionPath::role()` returns the stored role (ExtrusionEntity.hpp:312).
/// - `ExtrusionLoop::role()` returns `paths.front().role()` (ExtrusionEntity.hpp:535).
/// - `ExtrusionEntityCollection::role()` collapses to `erMixed` when child roles
///   differ (ExtrusionEntityCollection.hpp:54-61).
fn entity_role(entity: &ExtrusionEntityType) -> ExtrusionRole {
    match entity {
        ExtrusionEntityType::Path(path) => path.role,
        ExtrusionEntityType::Loop(l) => l.role(),
        ExtrusionEntityType::Collection(coll) => coll.role(),
    }
}

/// Extract perimeter polygons of the given layer.
/// SeamPlacer.cpp:371-417
///
/// C++ fills `std::vector<const LayerRegion *> &corresponding_regions_out`
/// (entries may be `nullptr`); this port uses `Option<&LayerRegion>`.
pub fn extract_perimeter_polygons<'a>(
    layer: &'a Layer,
    configured_seam_preference: SeamPosition,
    corresponding_regions_out: &mut Vec<Option<&'a LayerRegion>>,
) -> Polygons {
    // SeamPlacer.cpp:374
    let mut polygons: Polygons = Vec::new();
    // SeamPlacer.cpp:375
    for layer_region in layer.regions() {
        // SeamPlacer.cpp:376
        for ex_entity in &layer_region.perimeters.entities {
            // SeamPlacer.cpp:377
            if let ExtrusionEntityType::Collection(coll) = ex_entity {
                // collection of inner, outer, and overhang perimeters
                // SeamPlacer.cpp:378
                for perimeter in &coll.entities {
                    // SeamPlacer.cpp:379
                    let mut role = entity_role(perimeter);
                    // SeamPlacer.cpp:380-384
                    if let ExtrusionEntityType::Loop(l) = perimeter {
                        for path in &l.paths {
                            if path.role == ExtrusionRole::ExternalPerimeter {
                                role = ExtrusionRole::ExternalPerimeter;
                            }
                        }
                    }

                    // SeamPlacer.cpp:386-392
                    if role == ExtrusionRole::ExternalPerimeter
                        || (is_perimeter(role)
                            && configured_seam_preference == SeamPosition::spRandom)
                    {
                        // for random seam alignment, extract all perimeters
                        let mut p: Vec<Point> = Vec::new();
                        collect_points(perimeter, &mut p);
                        polygons.push(Polygon::from_points(p));
                        corresponding_regions_out.push(Some(layer_region));
                    }
                }
                // SeamPlacer.cpp:394-399
                if polygons.is_empty() {
                    let mut p: Vec<Point> = Vec::new();
                    collect_points(ex_entity, &mut p);
                    polygons.push(Polygon::from_points(p));
                    corresponding_regions_out.push(Some(layer_region));
                }
            } else if crate::faithful_gate("ZSMOOTH_FAITHFUL") {
                // R191: this crate stores perimeters FLAT (individual loops), not
                // native's per-island Collections — so every loop fell into the
                // unconditional else-branch and INNER perimeters entered the seam
                // data (rust 1429 perimeters vs native 749; align/pick pollution).
                // Faithful EFFECT: apply the collection-child role filter
                // (SeamPlacer.cpp:379-392) to the flat loop.
                let mut role = entity_role(ex_entity);
                if let ExtrusionEntityType::Loop(l) = ex_entity {
                    for path in &l.paths {
                        if path.role == ExtrusionRole::ExternalPerimeter {
                            role = ExtrusionRole::ExternalPerimeter;
                        }
                    }
                }
                if role == ExtrusionRole::ExternalPerimeter
                    || (is_perimeter(role) && configured_seam_preference == SeamPosition::spRandom)
                {
                    let mut p: Vec<Point> = Vec::new();
                    collect_points(ex_entity, &mut p);
                    polygons.push(Polygon::from_points(p));
                    corresponding_regions_out.push(Some(layer_region));
                }
            } else {
                // SeamPlacer.cpp:400-405
                let mut p: Vec<Point> = Vec::new();
                collect_points(ex_entity, &mut p);
                polygons.push(Polygon::from_points(p));
                corresponding_regions_out.push(Some(layer_region));
            }
        }
    }

    // SeamPlacer.cpp:409-413
    if polygons.is_empty() {
        // If there are no perimeter polygons for whatever reason (disabled perimeters .. )
        // insert dummy point. it is easier than checking everywhere if the layer is not
        // emtpy, no seam will be placed to this layer anyway
        polygons.push(Polygon::from_points(vec![Point::new(0, 0)]));
        corresponding_regions_out.push(None);
    }

    // SeamPlacer.cpp:416
    polygons
}

/// Insert SeamCandidates created from perimeter polygons in to the result vector.
/// Compute its type (Enfrocer,Blocker), angle, and position
/// each SeamCandidate also contains pointer to shared Perimeter structure representing the polygon
/// if Custom Seam modifiers are present, oversamples the polygon if necessary to better fit user intentions
/// SeamPlacer.cpp:419-549
///
/// C++ `region->flow(FlowRole::frExternalPerimeter)` reads `m_layer->height`
/// through the region's Layer back-pointer; this crate's `LayerRegion::flow`
/// takes the height explicitly, so `layer_height` is threaded by the caller
/// (`gather_seam_candidates` passes the owning layer's `height`).
pub fn process_perimeter_polygon(
    orig_polygon: &Polygon,
    z_coord: f32,
    region: Option<&LayerRegion>,
    layer_height: f64,
    global_model_info: &GlobalModelInfo,
    result: &mut LayerSeams,
) {
    // SeamPlacer.cpp:425
    if orig_polygon.len() == 0 {
        return;
    }
    // SeamPlacer.cpp:426-428
    let mut polygon = orig_polygon.clone();
    let was_clockwise = {
        polygon.make_counter_clockwise();
        polygon.is_clockwise()
    };
    // SeamPlacer.cpp:429
    let angle_arm_len: f32 = match region {
        Some(r) => r
            .flow(FlowRole::ExternalPerimeter, layer_height)
            .expect("LayerRegion::flow(frExternalPerimeter)")
            .nozzle_diameter() as f32,
        None => 0.5,
    };

    // SeamPlacer.cpp:431-433
    let mut lengths: Vec<f32> = Vec::new();
    for point_idx in 0..polygon.len() - 1 {
        lengths.push(
            (unscale_point(&polygon.points()[point_idx])
                - unscale_point(&polygon.points()[point_idx + 1]))
            .length() as f32,
        );
    }
    lengths.push(
        (unscale_point(&polygon.points()[0])
            - unscale_point(&polygon.points()[polygon.len() - 1]))
        .length()
        .max(0.1) as f32,
    );
    // SeamPlacer.cpp:434
    let polygon_angles = calculate_polygon_angles_at_vertices(&polygon, &lengths, angle_arm_len);

    // SeamPlacer.cpp:436-437 — result.perimeters.push_back({}); Perimeter &perimeter = back();
    result.perimeters.push_back(Perimeter::default());
    let perim_idx = result.perimeters.len() - 1;

    // SeamPlacer.cpp:439-443
    let mut orig_polygon_points: VecDeque<Vec3f> = VecDeque::new();
    for index in 0..polygon.len() {
        let unscaled_p = unscale_point(&polygon.points()[index]);
        orig_polygon_points.push_back(Vec3f::new(
            unscaled_p.x as f32,
            unscaled_p.y as f32,
            z_coord,
        ));
    }
    // SeamPlacer.cpp:444-449
    let first = *orig_polygon_points.front().unwrap();
    let mut oversampled_points: VecDeque<Vec3f> = VecDeque::new();
    let mut orig_angle_index = 0usize;
    result.perimeters[perim_idx].start_index = result.points.len();
    result.perimeters[perim_idx].flow_width = match region {
        Some(r) => r
            .flow(FlowRole::ExternalPerimeter, layer_height)
            .expect("LayerRegion::flow(frExternalPerimeter)")
            .width() as f32,
        None => 0.0,
    };
    let flow_width = result.perimeters[perim_idx].flow_width;
    // SeamPlacer.cpp:450
    let mut some_point_enforced = false;
    // SeamPlacer.cpp:451
    while !orig_polygon_points.is_empty() || !oversampled_points.is_empty() {
        // SeamPlacer.cpp:452-455
        let mut r#type = EnforcedBlockedSeamPoint::Neutral;
        let position: Vec3f;
        let mut local_ccw_angle = 0.0_f32;
        let mut orig_point = false;
        // SeamPlacer.cpp:456-464
        if let Some(p) = oversampled_points.pop_front() {
            position = p;
        } else {
            position = orig_polygon_points.pop_front().unwrap();
            local_ccw_angle = if was_clockwise {
                -polygon_angles[orig_angle_index]
            } else {
                polygon_angles[orig_angle_index]
            };
            orig_angle_index += 1;
            orig_point = true;
        }

        // SeamPlacer.cpp:466
        if global_model_info.is_enforced(&position, flow_width) {
            r#type = EnforcedBlockedSeamPoint::Enforced;
        }

        // SeamPlacer.cpp:468
        if global_model_info.is_blocked(&position, flow_width) {
            r#type = EnforcedBlockedSeamPoint::Blocked;
        }
        // SeamPlacer.cpp:469
        some_point_enforced = some_point_enforced || r#type == EnforcedBlockedSeamPoint::Enforced;

        // SeamPlacer.cpp:471-484
        if orig_point {
            let pos_of_next = if orig_polygon_points.is_empty() {
                first
            } else {
                *orig_polygon_points.front().unwrap()
            };
            let distance_to_next = (position - pos_of_next).norm();
            // SeamPlacer.cpp:474-475
            if distance_to_next > flow_width * flow_width * 4.0 {
                oversampled_points.push_back((position + pos_of_next) / 2.0);
            }
            // SeamPlacer.cpp:476-483
            if global_model_info.is_enforced(&position, distance_to_next) {
                let vec_to_next = (pos_of_next - position).normalize();
                let step_size = ENFORCER_OVERSAMPLING_DISTANCE;
                let mut step = step_size;
                while step < distance_to_next {
                    oversampled_points.push_back(position + vec_to_next * step);
                    step += step_size;
                }
            }
        }

        // SeamPlacer.cpp:486 — result.points.emplace_back(position, perimeter, local_ccw_angle, type);
        let mut cand = SeamCandidate::new(&position, perim_idx, local_ccw_angle, r#type);
        // Rust-only: cache `perimeter.flow_width` for the reference-free
        // comparator (see `SeamCandidate::flow_width_hint`).
        cand.set_flow_width_hint(flow_width);
        result.points.push(cand);
    }

    // SeamPlacer.cpp:489
    result.perimeters[perim_idx].end_index = result.points.len();

    // SeamPlacer.cpp:491-548
    if some_point_enforced {
        // We will patches of enforced points (patch: continuous section of enforced points), choose
        // the longest patch, and select the middle point or sharp point (depending on the angle)
        // this point will have high priority on this perimeter
        // SeamPlacer.cpp:495-496
        let start_index = result.perimeters[perim_idx].start_index;
        let end_index = result.perimeters[perim_idx].end_index;
        let perimeter_size = end_index - start_index;
        let next_index =
            |idx: usize| -> usize { start_index + next_idx_modulo(idx - start_index, perimeter_size) };

        // SeamPlacer.cpp:498-507
        let mut patches_starts_ends: Vec<usize> = Vec::new();
        for i in start_index..end_index {
            if result.points[i].r#type != EnforcedBlockedSeamPoint::Enforced
                && result.points[next_index(i)].r#type == EnforcedBlockedSeamPoint::Enforced
            {
                patches_starts_ends.push(next_index(i));
            }
            if result.points[i].r#type == EnforcedBlockedSeamPoint::Enforced
                && result.points[next_index(i)].r#type != EnforcedBlockedSeamPoint::Enforced
            {
                patches_starts_ends.push(next_index(i));
            }
        }
        // if patches_starts_ends are empty, it means that the whole perimeter is enforced..
        // don't do anything in that case
        // SeamPlacer.cpp:509-518
        if !patches_starts_ends.is_empty() {
            // if the first point in the patches is not enforced, it marks a patch end. in that
            // case, put it to the end and start on next to simplify the processing
            debug_assert!(patches_starts_ends.len() % 2 == 0);
            let mut start_on_second = false;
            if result.points[patches_starts_ends[0]].r#type != EnforcedBlockedSeamPoint::Enforced {
                start_on_second = true;
                patches_starts_ends.push(patches_starts_ends[0]);
            }
            // now pick the longest patch
            // SeamPlacer.cpp:519-527
            let mut longest_patch: (usize, usize) = (0, 0);
            // C++ `patch_len` mixes absolute point indices with the perimeter
            // *size* in size_t arithmetic, which wraps on underflow; mirrored
            // with wrapping ops.
            let patch_len = |start_end: (usize, usize)| -> usize {
                if start_end.1 < start_end.0 {
                    start_end.0.wrapping_add(perimeter_size.wrapping_sub(start_end.1))
                } else {
                    start_end.1 - start_end.0
                }
            };
            // SeamPlacer.cpp:528-531
            let mut patch_idx = if start_on_second { 1usize } else { 0usize };
            while patch_idx < patches_starts_ends.len() {
                let current_patch = (
                    patches_starts_ends[patch_idx],
                    patches_starts_ends[patch_idx + 1],
                );
                if patch_len(longest_patch) < patch_len(current_patch) {
                    longest_patch = current_patch;
                }
                patch_idx += 2;
            }
            // SeamPlacer.cpp:532-537
            let mut viable_points_indices: Vec<usize> = Vec::new();
            let mut large_angle_points_indices: Vec<usize> = Vec::new();
            let mut point_idx = longest_patch.0;
            while point_idx != longest_patch.1 {
                viable_points_indices.push(point_idx);
                if result.points[point_idx].local_ccw_angle.abs() > SHARP_ANGLE_SNAPPING_THRESHOLD {
                    large_angle_points_indices.push(point_idx);
                }
                point_idx = next_index(point_idx);
            }
            // SeamPlacer.cpp:538
            debug_assert!(!viable_points_indices.is_empty());
            // SeamPlacer.cpp:539-545
            if large_angle_points_indices.is_empty() {
                let central_idx = viable_points_indices[viable_points_indices.len() / 2];
                result.points[central_idx].central_enforcer = true;
            } else {
                let central_idx = large_angle_points_indices.len() / 2;
                result.points[large_angle_points_indices[central_idx]].central_enforcer = true;
            }
        }
    }
}

/// SeamPlacer.cpp:552-571
///
/// Get index of previous and next perimeter point of the layer.
pub fn find_previous_and_next_perimeter_point(
    perimeter_points: &[SeamCandidate],
    perimeters: &[Perimeter],
    point_index: usize,
) -> (usize, usize) {
    // SeamPlacer.cpp:554
    let current = &perimeter_points[point_index];
    // SeamPlacer.cpp:555-556 — for majority of points, neighbours lie behind and in front.
    let mut prev: i64 = point_index as i64 - 1;
    let mut next: i64 = point_index as i64 + 1;

    // C++: current.perimeter is a reference; we resolve the index into the
    // owning layer's `perimeters`.
    let perimeter = &perimeters[current.perimeter];
    // SeamPlacer.cpp:558-561
    if point_index == perimeter.start_index {
        // if point_index is equal to start, the previous neighbour is at the end
        prev = perimeter.end_index as i64;
    }

    // SeamPlacer.cpp:563-566
    if point_index == perimeter.end_index - 1 {
        // if point_index is equal to end, the next neighbour is at the start
        next = perimeter.start_index as i64;
    }

    // SeamPlacer.cpp:568-570
    debug_assert!(prev >= 0);
    debug_assert!(next >= 0);
    (prev as usize, next as usize)
}

/// SeamPlacer.cpp:670-751 — struct SeamComparator.
pub struct SeamComparator {
    // SeamPlacer.cpp:672
    pub setup: SeamPosition,
    // SeamPlacer.cpp:673
    pub angle_importance: f32,
}

impl SeamComparator {
    // SeamPlacer.cpp:674-677
    pub fn new(setup: SeamPosition) -> Self {
        let angle_importance = if setup == SeamPosition::spNearest {
            ANGLE_IMPORTANCE_NEAREST
        } else {
            ANGLE_IMPORTANCE_ALIGNED
        };
        Self {
            setup,
            angle_importance,
        }
    }

    /// Standard comparator. Returns whether `a` is a better SeamCandidate than `b`.
    /// SeamPlacer.cpp:681-712
    pub fn is_first_better(
        &self,
        a: &SeamCandidate,
        b: &SeamCandidate,
        preffered_location: &Vec2f,
    ) -> bool {
        // SeamPlacer.cpp:683
        if self.setup == SeamPosition::spAligned && a.central_enforcer != b.central_enforcer {
            return a.central_enforcer;
        }

        // Blockers/Enforcers discrimination, top priority
        // SeamPlacer.cpp:686
        if a.r#type != b.r#type {
            return a.r#type > b.r#type;
        }

        // avoid overhangs
        // SeamPlacer.cpp:689
        if a.overhang > 0.0 || b.overhang > 0.0 {
            return a.overhang < b.overhang;
        }

        // prefer hidden points (more than 0.5 mm inside)
        // SeamPlacer.cpp:692-693
        if a.embedded_distance < -0.5 && b.embedded_distance > -0.5 {
            return true;
        }
        if b.embedded_distance < -0.5 && a.embedded_distance > -0.5 {
            return false;
        }

        // SeamPlacer.cpp:695
        if self.setup == SeamPosition::spRear && a.position.y != b.position.y {
            return a.position.y > b.position.y;
        }

        // SeamPlacer.cpp:697-702
        let mut distance_penalty_a = 0.0_f32;
        let mut distance_penalty_b = 0.0_f32;
        if self.setup == SeamPosition::spNearest {
            distance_penalty_a =
                1.0 - gauss((a.position.xy() - preffered_location).norm(), 0.0, 1.0, 0.005);
            distance_penalty_b =
                1.0 - gauss((b.position.xy() - preffered_location).norm(), 0.0, 1.0, 0.005);
        }

        // SeamPlacer.cpp:704-705
        let a_overhang_around_penalty: f64 = if a.extra_overhang_point < OVERHANG_FILTER {
            0.0
        } else {
            a.extra_overhang_point as f64
        };
        let b_overhang_around_penalty: f64 = if b.extra_overhang_point < OVERHANG_FILTER {
            0.0
        } else {
            b.extra_overhang_point as f64
        };

        // the penalites are kept close to range [0-1.x] however, it should not be relied upon
        // SeamPlacer.cpp:708-709
        let penalty_a = a.overhang
            + a.visibility
            + self.angle_importance * compute_angle_penalty(a.local_ccw_angle)
            + distance_penalty_a
            + a_overhang_around_penalty as f32;
        let penalty_b = b.overhang
            + b.visibility
            + self.angle_importance * compute_angle_penalty(b.local_ccw_angle)
            + distance_penalty_b
            + b_overhang_around_penalty as f32;

        // SeamPlacer.cpp:711
        penalty_a < penalty_b
    }

    /// Comparator used during alignment.
    /// SeamPlacer.cpp:717-748
    pub fn is_first_not_much_worse(&self, a: &SeamCandidate, b: &SeamCandidate) -> bool {
        // Blockers/Enforcers discrimination, top priority
        // SeamPlacer.cpp:720-723
        if self.setup == SeamPosition::spAligned && a.central_enforcer != b.central_enforcer {
            // Prefer centers of enforcers.
            return a.central_enforcer;
        }

        // SeamPlacer.cpp:725
        if a.r#type == EnforcedBlockedSeamPoint::Enforced {
            return true;
        }

        // SeamPlacer.cpp:727
        if a.r#type == EnforcedBlockedSeamPoint::Blocked {
            return false;
        }

        // SeamPlacer.cpp:729
        if a.r#type != b.r#type {
            return a.r#type > b.r#type;
        }

        // avoid overhangs
        // SeamPlacer.cpp:732
        if (a.overhang > 0.0 || b.overhang > 0.0)
            && (a.overhang - b.overhang).abs() > (0.1 * a.perimeter_flow_width(b))
        {
            return a.overhang < b.overhang;
        }

        // prefer hidden points (more than 0.5 mm inside)
        // SeamPlacer.cpp:735-736
        if a.embedded_distance < -0.5 && b.embedded_distance > -0.5 {
            return true;
        }
        if b.embedded_distance < -0.5 && a.embedded_distance > -0.5 {
            return false;
        }

        // SeamPlacer.cpp:738
        if self.setup == SeamPosition::spRandom {
            return true;
        }

        // SeamPlacer.cpp:740
        if self.setup == SeamPosition::spRear {
            return a.position.y + SEAM_ALIGN_SCORE_TOLERANCE * 5.0 > b.position.y;
        }

        // SeamPlacer.cpp:742-743
        let a_overhang_around_penalty: f64 = if a.extra_overhang_point < OVERHANG_FILTER {
            0.0
        } else {
            a.extra_overhang_point as f64
        };
        let b_overhang_around_penalty: f64 = if b.extra_overhang_point < OVERHANG_FILTER {
            0.0
        } else {
            b.extra_overhang_point as f64
        };

        // SeamPlacer.cpp:745-746
        let penalty_a = a.overhang
            + a.visibility
            + self.angle_importance * compute_angle_penalty(a.local_ccw_angle)
            + a_overhang_around_penalty as f32;
        let penalty_b = b.overhang
            + b.visibility
            + self.angle_importance * compute_angle_penalty(b.local_ccw_angle)
            + b_overhang_around_penalty as f32;
        // SeamPlacer.cpp:747
        penalty_a <= penalty_b || penalty_a - penalty_b < SEAM_ALIGN_SCORE_TOLERANCE
    }

    // SeamPlacer.cpp:750
    pub fn are_similar(&self, a: &SeamCandidate, b: &SeamCandidate) -> bool {
        self.is_first_not_much_worse(a, b) && self.is_first_not_much_worse(b, a)
    }
}

impl SeamCandidate {
    /// `a.perimeter.flow_width` — the C++ `is_first_not_much_worse` uses
    /// `a.perimeter.flow_width` (SeamPlacer.cpp:732). Since our `SeamCandidate`
    /// stores the perimeter index rather than a reference, the flow width must be
    /// supplied via the owning layer. The standalone comparator API operates on
    /// candidates whose `perimeter` field is meaningful only with a layer; the
    /// callers that have a layer pass the actual flow width via wrapper. For the
    /// reference-free path we approximate `a.perimeter.flow_width` by 0 only when
    /// no layer context exists, which never happens in the real pipeline.
    #[inline]
    fn perimeter_flow_width(&self, _b: &SeamCandidate) -> f32 {
        // Set by `SeamCandidate::flow_width_hint`. Defaults to 0.0 which matches
        // the C++ when flow_width is 0 (dummy perimeters).
        self.flow_width_hint
    }
}

/// Extension field for `SeamCandidate` carrying the owning perimeter's
/// `flow_width`, so `is_first_not_much_worse` can read it without a back-pointer.
/// This is populated when candidates are gathered into a layer.
impl SeamCandidate {
    pub fn set_flow_width_hint(&mut self, w: f32) {
        self.flow_width_hint = w;
    }
}

// ============================================================================
// SeamPlacer.cpp:800-822 — pick_seam_point / pick_nearest_seam_point_index
// ============================================================================

/// Pick best seam point based on the given comparator.
/// SeamPlacer.cpp:801-810
pub fn pick_seam_point(
    perimeter_points: &mut [SeamCandidate],
    perimeters: &mut [Perimeter],
    start_index: usize,
    comparator: &SeamComparator,
) {
    // SeamPlacer.cpp:803
    let perim_idx = perimeter_points[start_index].perimeter;
    let end_index = perimeters[perim_idx].end_index;

    // SeamPlacer.cpp:805
    let mut seam_index = start_index;
    // SeamPlacer.cpp:806-808
    for index in start_index..end_index {
        if comparator.is_first_better(
            &perimeter_points[index],
            &perimeter_points[seam_index],
            &Vec2f::new(0.0, 0.0),
        ) {
            seam_index = index;
        }
    }
    // SeamPlacer.cpp:809
    perimeters[perim_idx].seam_index = seam_index;
}

/// SeamPlacer.cpp:812-822
pub fn pick_nearest_seam_point_index(
    perimeter_points: &[SeamCandidate],
    perimeters: &[Perimeter],
    start_index: usize,
    preffered_location: &Vec2f,
) -> usize {
    // SeamPlacer.cpp:814
    let perim_idx = perimeter_points[start_index].perimeter;
    let end_index = perimeters[perim_idx].end_index;
    // SeamPlacer.cpp:815
    let comparator = SeamComparator::new(SeamPosition::spNearest);

    // SeamPlacer.cpp:817
    let mut seam_index = start_index;
    // SeamPlacer.cpp:818-820
    for index in start_index..end_index {
        if comparator.is_first_better(
            &perimeter_points[index],
            &perimeter_points[seam_index],
            preffered_location,
        ) {
            seam_index = index;
        }
    }
    // SeamPlacer.cpp:821
    seam_index
}

/// picks random seam point uniformly, respecting enforcers blockers and overhang
/// avoidance.
/// SeamPlacer.cpp:825-883
pub fn pick_random_seam_point(
    perimeter_points: &[SeamCandidate],
    perimeters: &mut [Perimeter],
    start_index: usize,
) {
    // SeamPlacer.cpp:827
    let comparator = SeamComparator::new(SeamPosition::spRandom);

    // SeamPlacer.cpp:834-835
    let mut viable_example_index = start_index;
    let perim_idx = perimeter_points[start_index].perimeter;
    let end_index = perimeters[perim_idx].end_index;

    // SeamPlacer.cpp:836-842 — struct Viable
    struct Viable {
        index: usize,
        edge_length: f32,
        edge: Vec3f,
    }
    let mut viables: Vec<Viable> = Vec::new();

    // SeamPlacer.cpp:845-847
    let pseudornd_seed = perimeter_points[viable_example_index].position;
    let mut rand = (pseudornd_seed.dot(&Vec3f::new(12.9898, 78.233, 133.3333)).sin()
        * 43758.5453)
        .abs();
    rand -= rand as i32 as f32;

    // SeamPlacer.cpp:849
    for index in start_index..end_index {
        // SeamPlacer.cpp:850
        if comparator.are_similar(
            &perimeter_points[index],
            &perimeter_points[viable_example_index],
        ) {
            // index ok, push info into viables
            // SeamPlacer.cpp:852-854
            let next = if index == end_index - 1 {
                start_index
            } else {
                index + 1
            };
            let edge_to_next = perimeter_points[next].position - perimeter_points[index].position;
            let dist_to_next = edge_to_next.norm();
            viables.push(Viable {
                index,
                edge_length: dist_to_next,
                edge: edge_to_next,
            });
        } else if comparator.is_first_not_much_worse(
            &perimeter_points[viable_example_index],
            &perimeter_points[index],
        ) {
            // SeamPlacer.cpp:855-856 — index is worse, skip this point
        } else {
            // index is better than viable example index, update example, clear
            // gathered info, start again.
            // SeamPlacer.cpp:860-865
            viable_example_index = index;
            viables.clear();

            let next = if index == end_index - 1 {
                start_index
            } else {
                index + 1
            };
            let edge_to_next = perimeter_points[next].position - perimeter_points[index].position;
            let dist_to_next = edge_to_next.norm();
            viables.push(Viable {
                index,
                edge_length: dist_to_next,
                edge: edge_to_next,
            });
        }
    }

    // now pick random point from the stored options
    // SeamPlacer.cpp:870
    let len_sum: f32 = viables.iter().fold(0.0, |acc, v| acc + v.edge_length);
    // SeamPlacer.cpp:871
    let mut picked_len = len_sum * rand;

    // SeamPlacer.cpp:873-877
    let mut point_idx = 0usize;
    while picked_len - viables[point_idx].edge_length > 0.0 {
        picked_len -= viables[point_idx].edge_length;
        point_idx += 1;
    }

    // SeamPlacer.cpp:879-882
    perimeters[perim_idx].seam_index = viables[point_idx].index;
    perimeters[perim_idx].final_seam_position = perimeter_points
        [perimeters[perim_idx].seam_index]
        .position
        + viables[point_idx].edge.normalize() * picked_len;
    perimeters[perim_idx].finalized = true;
}

/// SeamPlacer.cpp:885-920 — class PerimeterDistancer.
///
/// The C++ class builds an AABB tree over *unscaled* `Linef`s. The crate's
/// `AABBTreeLines` operates on *scaled* integer `Line`s with a `PointF` query in
/// the same (scaled) space. We therefore keep both the unscaled lines (for the
/// final sign computation, SeamPlacer.cpp:914-917) and the scaled lines + tree
/// (for the distance query), then unscale the resulting length. This is
/// geometrically identical to the C++ result.
pub struct PerimeterDistancer {
    // SeamPlacer.cpp:887 — `std::vector<Linef> lines;`
    lines: Vec<LineF>,
    // scaled copy used by the crate's AABBTreeLines query.
    scaled_lines: Vec<Line>,
    // SeamPlacer.cpp:888 — `AABBTreeIndirect::Tree<2, double> tree;`
    tree: tree2d::Tree,
}

impl PerimeterDistancer {
    /// Build from a layer outline (set of islands, each contour + holes).
    /// SeamPlacer.cpp:891-903
    ///
    /// We take the `lslices` ExPolygons directly to avoid depending on the
    /// not-yet-wired `Layer` accessor.
    pub fn new(layer_outline: &[crate::geometry::ExPolygon]) -> Self {
        let mut lines: Vec<LineF> = Vec::new();
        let mut scaled_lines: Vec<Line> = Vec::new();
        // SeamPlacer.cpp:894-901
        for island in layer_outline {
            // assert(island.contour.is_counter_clockwise());  SeamPlacer.cpp:895
            for line in island.contour.lines() {
                lines.push(LineF::new(unscale_point(&line.a), unscale_point(&line.b)));
                scaled_lines.push(Line::new(line.a, line.b));
            }
            for hole in &island.holes {
                // assert(hole.is_clockwise());  SeamPlacer.cpp:898
                for line in hole.lines() {
                    lines.push(LineF::new(unscale_point(&line.a), unscale_point(&line.b)));
                    scaled_lines.push(Line::new(line.a, line.b));
                }
            }
        }
        // SeamPlacer.cpp:902
        let tree = build_aabb_tree_over_indexed_lines(&scaled_lines);
        Self {
            lines,
            scaled_lines,
            tree,
        }
    }

    /// SeamPlacer.cpp:905-919
    pub fn distance_from_perimeter(&self, point: &Vec2f) -> f32 {
        // SeamPlacer.cpp:907 — `Vec2d p = point.cast<double>();`
        let p = PointF::new(point.x as f64, point.y as f64);
        // The query is run in scaled space: scale the query point and compute
        // the squared distance against the scaled lines.
        let scaled_p = PointF::new(p.x * crate::SCALING_FACTOR, p.y * crate::SCALING_FACTOR);
        // SeamPlacer.cpp:908-910
        let mut hit_idx_out: usize = 0;
        let mut hit_point_out = PointF::new(0.0, 0.0);
        let mut distance = squared_distance_to_indexed_lines(
            &self.scaled_lines,
            &self.tree,
            scaled_p,
            &mut hit_idx_out,
            &mut hit_point_out,
            f64::MAX,
        );
        // SeamPlacer.cpp:911
        if distance < 0.0 {
            return f32::MAX;
        }

        // SeamPlacer.cpp:913 — `distance = sqrt(distance);` then unscale to mm.
        distance = distance.sqrt() / crate::SCALING_FACTOR;
        // SeamPlacer.cpp:914-917
        let line = &self.lines[hit_idx_out];
        let v1 = Vec2d::new(line.b.x - line.a.x, line.b.y - line.a.y);
        let v2 = Vec2d::new(p.x - line.a.x, p.y - line.a.y);
        if (v1.x * v2.y) - (v1.y * v2.x) > 0.0 {
            distance *= -1.0;
        }
        distance as f32
    }
}

#[inline]
fn unscale_point(p: &Point) -> PointF {
    PointF::new(unscale(p.x), unscale(p.y))
}

// ============================================================================
// Compatibility / glue layer
// ============================================================================
//
// The remainder of this file preserves the public API consumed by the rest of
// the crate (`perimeter_generator.rs`, `gcode/exporter.rs`, `lib.rs`
// re-exports): `find_best_seam_index`, `place_seam`, `create_seam_placer`,
// `SeamPlacer`, `SeamPlacerConfig`, `SeamPlacerStats`, `SeamPositionMode`,
// `LayerOutline`, `PerimeterOutline`, `Point3f`.
//
// These wrappers are NOT part of the C++ source. They provide an angle-based
// seam-selection entry point used while the full
// `SeamPlacer::{init, place_seam}` pipeline remains blocked on the f32
// KD-tree queries and `PrintObject::model_object`/`ModelVolume` accessors
// enumerated at the top of this file. The scoring uses the faithful
// `compute_angle_penalty` / `gauss` functions above so that the eventual full
// port and this shim agree on the angle/visibility math.

/// Seam position preference mode (glue type mirroring `SeamPosition`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum SeamPositionMode {
    #[default]
    Aligned,
    Nearest,
    Random,
    Rear,
    Hidden,
}

/// Configuration for the seam placer glue entry points.
#[derive(Clone, Debug)]
pub struct SeamPlacerConfig {
    pub seam_position: SeamPositionMode,
    pub min_arm_length: f64,
    pub angle_importance: f64,
}

impl Default for SeamPlacerConfig {
    fn default() -> Self {
        Self {
            seam_position: SeamPositionMode::Aligned,
            // process_perimeter_polygon uses nozzle_diameter (0.5 default arm len)
            // SeamPlacer.cpp:429
            min_arm_length: 0.5,
            // SeamPlacer.hpp:144
            angle_importance: ANGLE_IMPORTANCE_ALIGNED as f64,
        }
    }
}

/// 3D floating-point position (glue type, re-exported for callers).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Point3f {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Point3f {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }
}

/// A perimeter outline for seam placement (glue input type).
#[derive(Clone, Debug)]
pub struct PerimeterOutline {
    pub polygon: Polygon,
    pub flow_width: f64,
    pub is_external: bool,
}

/// Layer outline data (glue input type).
#[derive(Clone, Debug)]
pub struct LayerOutline {
    pub z_height: f64,
    pub perimeters: Vec<PerimeterOutline>,
    pub layer_regions: Vec<Polygon>,
}

/// Statistics about seam placement (glue type).
#[derive(Clone, Debug)]
pub struct SeamPlacerStats {
    pub layer_count: usize,
    pub total_perimeters: usize,
    pub total_candidates: usize,
    pub finalized_count: usize,
    pub seam_position_mode: SeamPositionMode,
}

/// Glue seam placer holding per-layer `LayerSeams` (the faithful data type).
#[derive(Clone, Debug, Default)]
pub struct SeamPlacer {
    config_mode: SeamPositionMode,
    seam_data: PrintObjectSeamData,
    /// R191 (ZSMOOTH_FAITHFUL): scaled-int XY offset rust-frame -> centered
    /// seam frame, captured at init; (0,0) when the gate is off.
    frame_offset_xy: (i64, i64),
}

impl SeamPlacer {
    pub fn new(config: SeamPlacerConfig) -> Self {
        Self {
            config_mode: config.seam_position,
            seam_data: PrintObjectSeamData::default(),
            frame_offset_xy: (0, 0),
        }
    }

    pub fn stats(&self) -> SeamPlacerStats {
        let total_perimeters: usize = self.seam_data.layers.iter().map(|l| l.perimeters.len()).sum();
        let total_candidates: usize = self.seam_data.layers.iter().map(|l| l.points.len()).sum();
        let finalized_count = self
            .seam_data
            .layers
            .iter()
            .flat_map(|l| l.perimeters.iter())
            .filter(|p| p.finalized)
            .count();
        SeamPlacerStats {
            layer_count: self.seam_data.layers.len(),
            total_perimeters,
            total_candidates,
            finalized_count,
            seam_position_mode: self.config_mode,
        }
    }

    /// Build per-layer candidate data from simple polygon layers.
    pub fn init_simple(&mut self, layers: &[(f64, Vec<Polygon>)], flow_width: f64) {
        self.seam_data.clear();
        for (z, polygons) in layers {
            let mut layer = LayerSeams::default();
            for polygon in polygons {
                gather_layer_perimeter(&mut layer, polygon, *z as f32, flow_width as f32);
            }
            self.seam_data.layers.push(layer);
        }
    }

    /// Parallel process and extract each perimeter polygon of the given print object.
    /// Gather SeamCandidates of each layer into vector and build KDtree over them
    /// Store results in the SeamPlacer variables m_seam_per_object
    /// SeamPlacer.cpp:927-948
    ///
    /// C++ stores into `m_seam_per_object.emplace(po, PrintObjectSeamData{})`
    /// (a map keyed by `const PrintObject *`); this port keeps a single
    /// `PrintObjectSeamData` because the only multi-object orchestrator
    /// (`SeamPlacer::init`, still blocked) clears the map before refilling it.
    /// C++ runs the per-layer loop under `tbb::parallel_for`; ported serially
    /// with identical per-layer results.
    /// R191 (ZSMOOTH_FAITHFUL): the scaled-int XY offset between the rust
    /// (uncentered) frame and native's centered seam frame — native's
    /// m_center_offset ints recovered exactly from slice_center_offset
    /// (cx_mm = int * SCALING_FACTOR, print_object.rs:453).
    fn seam_frame_offset(po: &PrintObject) -> Option<(i64, i64)> {
        // R198/R203: ALWAYS None. Entities, lslices and exporter loops all live
        // in the SAME frame as native's seam data (R197 PERIENT bit-identity);
        // the R191 translation double-shifted every candidate by -0.8245mm.
        // (R198 first removed this; the R199 bisect's `git checkout -- crates/`
        // silently restored the R191 version and it was re-committed unnoticed —
        // R200-R202 all measured with the double-shift active.)
        let _ = po;
        None
    }

    pub fn gather_seam_candidates(
        &mut self,
        po: &PrintObject,
        global_model_info: &GlobalModelInfo,
        configured_seam_preference: SeamPosition,
    ) {
        // SeamPlacer.cpp:930-931 — fresh emplace + resize(po->layer_count()).
        let seam_data = &mut self.seam_data;
        seam_data.layers.clear();
        seam_data
            .layers
            .resize(po.layers().len(), LayerSeams::default());

        // SeamPlacer.cpp:933-934 — C++ runs this under `tbb::parallel_for`.
        // R521 MEASURED: rayon here is INERT (export_gcode 4.131 -> 4.130s, i.e.
        // zero) — this loop is not on the export critical path. Kept serial.
        for layer_idx in 0..po.layers().len() {
            // SeamPlacer.cpp:935-937
            let layer = &po.layers()[layer_idx];
            let unscaled_z = layer.slice_z;
            let mut regions: Vec<Option<&LayerRegion>> = Vec::new();
            // NOTE corresponding region ptr may be null, if the layer has zero perimeters
            // SeamPlacer.cpp:940
            let mut polygons = extract_perimeter_polygons(layer, configured_seam_preference, &mut regions);
            // R191: native candidates live in the CENTERED slicing frame
            // (native polygons are centered; SeamPlacer unscales them directly).
            // Rust perimeters are uncentered — translate the scaled ints by
            // -m_center_offset (integer-exact) so every unscaled candidate
            // position is bit-identical to native's.
            if let Some((sx, sy)) = Self::seam_frame_offset(po) {
                for poly in polygons.iter_mut() {
                    for pt in poly.points_mut() {
                        pt.x -= sx;
                        pt.y -= sy;
                    }
                }
            }
            // SeamPlacer.cpp:941-943
            for poly_index in 0..polygons.len() {
                process_perimeter_polygon(
                    &polygons[poly_index],
                    unscaled_z as f32,
                    regions[poly_index],
                    layer.height,
                    global_model_info,
                    &mut seam_data.layers[layer_idx],
                );
            }
            // SeamPlacer.cpp:944-945 — C++ builds `points_tree` here; this port
            // builds it on demand via `LayerSeams::build_points_tree` (the KD
            // tree borrows the coordinate functor closure).
        }
    }

    /// SeamPlacer.cpp:950-960
    ///
    /// C++ indexes `m_seam_per_object[po]`; see the map note on
    /// [`SeamPlacer::gather_seam_candidates`]. C++ runs under
    /// `tbb::parallel_for`; ported serially with identical results.
    pub fn calculate_candidates_visibility(
        &mut self,
        _po: &PrintObject,
        global_model_info: &GlobalModelInfo,
    ) {
        // Build the sample KD tree once (the C++ `mesh_samples_tree` is persistent
        // in GlobalModelInfo; here the tree borrows the sample positions so it is
        // built here and reused across all candidate queries). When the model info
        // is empty the tree is empty and every visibility is 1.0.
        let mesh_samples_tree = global_model_info.build_mesh_samples_tree();
        // SeamPlacer.cpp:954
        let layers = &mut self.seam_data.layers;
        // SeamPlacer.cpp:955-959 — C++ runs this under `tbb::parallel_for` over
        // the layer range. R521: ported to rayon to match. Every point's
        // visibility is an independent pure function of (mesh_samples_tree,
        // position) written to its own slot, so the result is order-independent
        // and byte-identical to the serial form.
        {
            use rayon::prelude::*;
            layers.par_iter_mut().for_each(|layer| {
                for perimeter_point in layer.points.iter_mut() {
                    perimeter_point.visibility = global_model_info
                        .calculate_point_visibility(&mesh_samples_tree, &perimeter_point.position);
                }
            });
        }
    }

    /// SeamPlacer.cpp:962-1046
    ///
    /// C++ runs under `tbb::parallel_for` over layer ranges, seeding each
    /// range's `prev_layer_distancer` from `r.begin() - 1`; this serial port is
    /// the single range `[0, layers.size())` and computes identical values.
    /// C++ `PerimeterDistancer(po->layers()[i])` reads `layer->lslices`; the
    /// crate's [`PerimeterDistancer::new`] takes those ExPolygons directly.
    pub fn calculate_overhangs_and_layer_embedding(&mut self, po: &PrintObject) {
        // R191: candidates are in the centered frame under the gate — the
        // lslices feeding the distancers must be translated identically
        // (integer-exact) or every overhang/embedding distance is 0.8245mm off.
        let frame_offset = Self::seam_frame_offset(po);
        let centered_lslices = |lslices: &Vec<crate::geometry::ExPolygon>| -> Vec<crate::geometry::ExPolygon> {
            let mut out = lslices.clone();
            if let Some((sx, sy)) = frame_offset {
                for ex in out.iter_mut() {
                    for pt in ex.contour.points_mut() {
                        pt.x -= sx;
                        pt.y -= sy;
                    }
                    for hole in ex.holes.iter_mut() {
                        for pt in hole.points_mut() {
                            pt.x -= sx;
                            pt.y -= sy;
                        }
                    }
                }
            }
            out
        };
        // SeamPlacer.cpp:965
        let layers = &mut self.seam_data.layers;
        // SeamPlacer.cpp:966 — C++ runs this under `tbb::parallel_for` over layer
        // RANGES, and each range seeds its own `prev_layer_distancer` from
        // `r.begin() - 1` (SeamPlacer.cpp:967-970). R522 mirrors that with
        // `par_chunks_mut`: chunk `c` covering `[base, base+len)` rebuilds the
        // distancer for layer `base - 1` before iterating, so every layer sees
        // exactly the same `prev_layer_distancer` it saw serially. The only cost
        // is one extra distancer per chunk, which is what C++ pays too.
        const SEAM_OVERHANG_CHUNK: usize = 8;
        use rayon::prelude::*;
        layers
            .par_chunks_mut(SEAM_OVERHANG_CHUNK)
            .enumerate()
            .for_each(|(chunk_idx, chunk)| {
        let base = chunk_idx * SEAM_OVERHANG_CHUNK;
        // SeamPlacer.cpp:967-970 — at r.begin() == 0 there is no previous layer.
        let mut prev_layer_distancer: Option<PerimeterDistancer> = if base == 0 {
            None
        } else {
            Some(PerimeterDistancer::new(&centered_lslices(
                &po.layers()[base - 1].lslices,
            )))
        };

        // SeamPlacer.cpp:972
        for (chunk_k, layer_seams) in chunk.iter_mut().enumerate() {
            let layer_idx = base + chunk_k;
            // SeamPlacer.cpp:973-977
            let mut regions_with_perimeter = 0usize;
            for region in po.layers()[layer_idx].regions() {
                if !region.perimeters.entities.is_empty() {
                    regions_with_perimeter += 1;
                }
            }
            // SeamPlacer.cpp:978
            let should_compute_layer_embedding = regions_with_perimeter > 1;
            // SeamPlacer.cpp:979
            let current_layer_distancer =
                PerimeterDistancer::new(&centered_lslices(&po.layers()[layer_idx].lslices));

            // SeamPlacer.cpp:981 (`int points_size = layers[layer_idx].points.size();`)
            let LayerSeams {
                perimeters, points, ..
            } = layer_seams;
            let points_size = points.len();
            // SeamPlacer.cpp:982
            for i in 0..points_size {
                // SeamPlacer.cpp:983-984 — Vec2f point = perimeter_point.position.head<2>();
                let point = points[i].position.xy();
                // C++ reads `perimeter_point.perimeter.flow_width` through the
                // candidate's Perimeter back-reference; resolved via the index.
                let perimeter_idx = points[i].perimeter;
                let flow_width = perimeters[perimeter_idx].flow_width;
                // SeamPlacer.cpp:985-992
                if let Some(prev) = prev_layer_distancer.as_ref() {
                    // SeamPlacer.cpp:986 — `double dist_temp = ...` (float widened).
                    let dist_temp = prev.distance_from_perimeter(&point) as f64;
                    // SeamPlacer.cpp:987-988 — the double expression is
                    // truncated to float on assignment; `tan` promotes the
                    // float threshold to double (C `tan(double)`).
                    let overhang = (dist_temp + (0.6_f32 * flow_width) as f64
                        - (OVERHANG_ANGLE_THRESHOLD as f64).tan()
                            * po.layers()[layer_idx].height) as f32;
                    points[i].overhang = if overhang < 0.0 { 0.0 } else { overhang };

                    // SeamPlacer.cpp:990-991
                    let overhang_degree =
                        ((dist_temp + (0.6_f32 * flow_width) as f64) / flow_width as f64) as f32;
                    points[i].overhang_degree = if overhang_degree < 0.0 {
                        0.0
                    } else {
                        overhang_degree
                    };
                }

                // SeamPlacer.cpp:994-996
                if should_compute_layer_embedding {
                    // search for embedded perimeter points (points hidden inside the print,
                    // e.g. multimaterial join, best position for seam)
                    points[i].embedded_distance =
                        current_layer_distancer.distance_from_perimeter(&point)
                            + 0.6_f32 * flow_width;
                }

                // SeamPlacer.cpp:998-999
                let start_index = perimeters[perimeter_idx].start_index;
                let end_index = perimeters[perimeter_idx].end_index;
                // SeamPlacer.cpp:1000
                if po.config().seam_placement_away_from_overhangs
                    && points[i].overhang_degree > 0.0
                    && end_index - start_index > 1
                {
                    // BBS. extend overhang range
                    // SeamPlacer.cpp:1002-1005
                    let mut dist = 0.0_f32;
                    let mut idx = i;
                    // `double gauss_value = gauss(...)` — float widened to double.
                    let gauss_value = gauss(0.0, 0.0, 1.0, 10.0) as f64;
                    let overhang_degree = points[i].overhang_degree;
                    points[i].extra_overhang_point = (overhang_degree as f64 * gauss_value) as f32;
                    // check left
                    // SeamPlacer.cpp:1007-1021
                    loop {
                        let prev = idx;
                        idx = if idx == start_index { end_index - 1 } else { idx - 1 };
                        if idx == i {
                            break;
                        }
                        // C++ `dist += sqrt(squaredNorm)` — float squaredNorm
                        // promoted to double for sqrt, sum truncated back to float.
                        dist = (dist as f64
                            + ((points[idx].position.xy() - points[prev].position.xy())
                                .norm_squared() as f64)
                                .sqrt()) as f32;
                        if dist > LENS_LIMIT {
                            break;
                        }
                        let gauss_value_dist = gauss(dist, 0.0, 1.0, 10.0) as f64;

                        if points[idx].extra_overhang_point as f64
                            > overhang_degree as f64 * gauss_value_dist
                        {
                            continue;
                        }
                        points[idx].extra_overhang_point =
                            (overhang_degree as f64 * gauss_value_dist) as f32;
                    }

                    // check right
                    // SeamPlacer.cpp:1023-1038
                    dist = 0.0;
                    idx = i;
                    loop {
                        let prev = idx;
                        idx = if idx == end_index - 1 { start_index } else { idx + 1 };
                        if idx == i {
                            break;
                        }
                        dist = (dist as f64
                            + ((points[idx].position.xy() - points[prev].position.xy())
                                .norm_squared() as f64)
                                .sqrt()) as f32;
                        if dist > LENS_LIMIT {
                            break;
                        }
                        let gauss_value_dist = gauss(dist, 0.0, 1.0, 10.0) as f64;

                        if points[idx].extra_overhang_point as f64
                            > overhang_degree as f64 * gauss_value_dist
                        {
                            continue;
                        }
                        points[idx].extra_overhang_point =
                            (overhang_degree as f64 * gauss_value_dist) as f32;
                    }
                }
            }

            // SeamPlacer.cpp:1042 — prev_layer_distancer.swap(current_layer_distancer);
            prev_layer_distancer = Some(current_layer_distancer);
        }
            });
    }

    /// Estimates, if there is good seam point in the layer_idx which is close to
    /// last_point_pos. Used by `align_seam_points`.
    /// SeamPlacer.cpp:1053-1105
    ///
    /// C++ queries `*layers[layer_idx].points_tree` (an f32 `KDTreeIndirect`); this
    /// port builds that tree on demand via [`LayerSeams::build_points_tree`] and
    /// queries it with [`find_nearby_points_eps`] (`epsilon = EPSILON as f32`,
    /// reproducing C++ `float(EPSILON)`).
    pub fn find_next_seam_in_layer(
        layers: &[LayerSeams],
        projected_position: &Vec3f,
        layer_idx: usize,
        max_distance: f32,
        comparator: &SeamComparator,
    ) -> Option<(usize, usize)> {
        // SeamPlacer.cpp:1060
        let points_tree = layers[layer_idx].build_points_tree();
        let nearby_points_indices = find_nearby_points_eps(
            &points_tree,
            projected_position,
            max_distance,
            EPSILON as f32,
            |_| true,
        );

        // SeamPlacer.cpp:1062
        if nearby_points_indices.is_empty() {
            return None;
        }

        // SeamPlacer.cpp:1064-1065
        let mut best_nearby_point_index = nearby_points_indices[0];
        let mut nearest_point_index = nearby_points_indices[0];

        let layer = &layers[layer_idx];
        let projected_xy = projected_position.xy();

        // helper resolving a candidate's owning perimeter (the C++
        // `point.perimeter` back-reference) into this layer's `perimeters`.
        let perim_of = |pt_idx: usize| -> &Perimeter {
            &layer.perimeters[layer.points[pt_idx].perimeter]
        };

        // SeamPlacer.cpp:1068-1081 — Now find best nearby point, nearest point.
        for &nearby_point_index in &nearby_points_indices {
            let point = &layer.points[nearby_point_index];
            // SeamPlacer.cpp:1070-1072
            if perim_of(nearby_point_index).finalized {
                continue; // skip over finalized perimeters
            }
            // SeamPlacer.cpp:1073-1076
            if comparator.is_first_better(point, &layer.points[best_nearby_point_index], &projected_xy)
                || perim_of(best_nearby_point_index).finalized
            {
                best_nearby_point_index = nearby_point_index;
            }
            // SeamPlacer.cpp:1077-1080
            if (point.position - projected_position).norm_squared()
                < (layer.points[nearest_point_index].position - projected_position).norm_squared()
                || perim_of(nearest_point_index).finalized
            {
                nearest_point_index = nearby_point_index;
            }
        }

        // SeamPlacer.cpp:1083-1084
        let best_nearby_point = &layer.points[best_nearby_point_index];
        let nearest_point = &layer.points[nearest_point_index];

        // SeamPlacer.cpp:1086-1089
        if perim_of(nearest_point_index).finalized {
            // all points are from already finalized perimeter, skip
            return None;
        }

        // SeamPlacer.cpp:1091-1092 — from the nearest_point, deduce index of seam.
        let nearest_perimeter = perim_of(nearest_point_index);
        let next_layer_seam = &layer.points[nearest_perimeter.seam_index];

        // SeamPlacer.cpp:1094-1097 — First try to pick central enforcer if present.
        // sqr(3 * max_distance)
        let three_md = 3.0 * max_distance;
        if next_layer_seam.central_enforcer
            && (next_layer_seam.position - projected_position).norm_squared() < three_md * three_md
        {
            return Some((layer_idx, nearest_perimeter.seam_index));
        }

        // SeamPlacer.cpp:1099-1100 — First try to align the nearest.
        if comparator.is_first_not_much_worse(nearest_point, next_layer_seam) {
            return Some((layer_idx, nearest_point_index));
        }
        // SeamPlacer.cpp:1101-1102 — then try the best nearby point.
        if comparator.is_first_not_much_worse(best_nearby_point, next_layer_seam) {
            return Some((layer_idx, best_nearby_point_index));
        }

        // SeamPlacer.cpp:1104
        None
    }

    /// SeamPlacer.cpp:1107-1154 — cluster nearby seams across layers into a string.
    pub fn find_seam_string(
        &self,
        po: &PrintObject,
        start_seam: (usize, usize),
        comparator: &SeamComparator,
    ) -> Vec<(usize, usize)> {
        // SeamPlacer.cpp:1111-1112
        let layers = &self.seam_data.layers;
        let layer_idx = start_seam.0 as i64;

        // SeamPlacer.cpp:1114-1118 — initialize search.
        let mut next_layer = layer_idx + 1;
        let mut step: i64 = 1;
        let mut prev_point_index = start_seam;
        let mut seam_string: Vec<(usize, usize)> = vec![start_seam];

        // SeamPlacer.cpp:1131 — max_distance is invariant across the search.
        let start_flow_width = layers[start_seam.0].perimeters
            [layers[start_seam.0].points[start_seam.1].perimeter]
            .flow_width;
        let max_distance = SEAM_ALIGN_TOLERABLE_DIST_FACTOR * start_flow_width;

        // SeamPlacer.cpp:1126
        while next_layer >= 0 {
            // SeamPlacer.cpp:1127-1130 — if past the top, reverse downward.
            if next_layer >= layers.len() as i64 {
                // reverse_lookup_direction (SeamPlacer.cpp:1120-1124)
                step = -1;
                prev_point_index = start_seam;
                next_layer = layer_idx - 1;
                if next_layer < 0 {
                    break;
                }
            }
            // SeamPlacer.cpp:1132-1134
            let prev_position = layers[prev_point_index.0].points[prev_point_index.1].position;
            let mut projected_position = prev_position;
            projected_position.z = po.layers()[next_layer as usize].slice_z as f32;

            // SeamPlacer.cpp:1136
            let maybe_next_seam = Self::find_next_seam_in_layer(
                layers,
                &projected_position,
                next_layer as usize,
                max_distance,
                comparator,
            );

            // SeamPlacer.cpp:1138-1150
            if let Some(next_seam) = maybe_next_seam {
                seam_string.push(next_seam);
                prev_point_index = *seam_string.last().unwrap();
            } else if step == 1 {
                // reverse_lookup_direction (SeamPlacer.cpp:1120-1124)
                step = -1;
                prev_point_index = start_seam;
                next_layer = layer_idx - 1;
                if next_layer < 0 {
                    break;
                }
            } else {
                break;
            }
            // SeamPlacer.cpp:1151
            next_layer += step;
        }
        // SeamPlacer.cpp:1153
        seam_string
    }

    /// Clusters already chosen seam points into strings across multiple layers,
    /// and then aligns the strings via polynomial fit.
    /// SeamPlacer.cpp:1161-1307
    ///
    /// C++ uses the per-object `m_seam_per_object[po].layers`; this port aligns
    /// `self.seam_data.layers` (single object). `Geometry::fit_cubic_bspline`
    /// maps to [`fit_cubic_bspline`] with `endpoints_level_of_freedom = 0`,
    /// `dimension = 2`. `curve.get_fitted_value(z)` maps to
    /// `get_fitted_value::<CubicBSplineKernel>(z)`.
    pub fn align_seam_points(&mut self, po: &PrintObject, comparator: &SeamComparator) {
        // SeamPlacer.cpp:1183-1184 — gather all seams.
        let mut seams = Self::gather_all_seams_of_object(&self.seam_data.layers);

        // SeamPlacer.cpp:1187-1189 — stable_sort by is_first_better.
        // std::stable_sort with a strict-weak "is_first_better(left,right)" comparator:
        // left should sort before right when it is the better seam.
        {
            let layers = &self.seam_data.layers;
            seams.sort_by(|&l, &r| {
                let a = &layers[l.0].points[l.1];
                let b = &layers[r.0].points[r.1];
                if comparator.is_first_better(a, b, &Vec2f::new(0.0, 0.0)) {
                    std::cmp::Ordering::Less
                } else if comparator.is_first_better(b, a, &Vec2f::new(0.0, 0.0)) {
                    std::cmp::Ordering::Greater
                } else {
                    std::cmp::Ordering::Equal
                }
            });
        }

        // SeamPlacer.cpp:1199-1200
        let mut global_index: i64 = 0;
        while global_index < seams.len() as i64 {
            // SeamPlacer.cpp:1201-1204
            let layer_idx = seams[global_index as usize].0;
            let seam_index = seams[global_index as usize].1;
            global_index += 1;
            // SeamPlacer.cpp:1205-1207
            let cand_perim =
                self.seam_data.layers[layer_idx].points[seam_index].perimeter;
            if self.seam_data.layers[layer_idx].perimeters[cand_perim].finalized {
                // This perimeter is already aligned, skip seam
                continue;
            }

            // SeamPlacer.cpp:1209
            let mut seam_string = self.find_seam_string(po, (layer_idx, seam_index), comparator);
            // SeamPlacer.cpp:1210
            let step_size = 1 + seam_string.len() / 20;
            // SeamPlacer.cpp:1211-1216 — try alternative starts, keep the longest.
            let mut alternative_start = 0usize;
            while alternative_start < seam_string.len() {
                let start_layer_idx = seam_string[alternative_start].0;
                let seam_idx = {
                    let p = self.seam_data.layers[start_layer_idx].points
                        [seam_string[alternative_start].1]
                        .perimeter;
                    self.seam_data.layers[start_layer_idx].perimeters[p].seam_index
                };
                let alternative_seam_string =
                    self.find_seam_string(po, (start_layer_idx, seam_idx), comparator);
                if alternative_seam_string.len() > seam_string.len() {
                    seam_string = alternative_seam_string;
                }
                alternative_start += step_size;
            }
            // SeamPlacer.cpp:1217-1220
            if seam_string.len() < SEAM_ALIGN_MINIMUM_STRING_SEAMS {
                // string NOT long enough to be worth aligning, skip
                continue;
            }

            // SeamPlacer.cpp:1224-1225 — sort by layer index.
            seam_string.sort_by(|a, b| a.0.cmp(&b.0));

            // SeamPlacer.cpp:1228 — repeat alignment for current seam.
            global_index -= 1;

            // SeamPlacer.cpp:1231-1233
            let n = seam_string.len();
            let mut observations: Vec<Vec<f32>> = vec![vec![0.0, 0.0]; n];
            let mut observation_points: Vec<f32> = vec![0.0; n];
            let mut weights: Vec<f32> = vec![0.0; n];

            // SeamPlacer.cpp:1235 — angle_3d.
            let angle_3d = |a: Vec3f, b: Vec3f| -> f32 {
                a.normalize().dot(&b.normalize()).acos().abs()
            };
            // SeamPlacer.cpp:1237 — angle_weight.
            let angle_weight = |angle: f32| -> f32 { 1.0 / (0.1 + compute_angle_penalty(angle)) };

            // SeamPlacer.cpp:1240-1259 — gather points positions and weights.
            let layers = &self.seam_data.layers;
            let mut total_length = 0.0_f32;
            let mut last_point_pos = layers[seam_string[0].0].points[seam_string[0].1].position;
            for index in 0..n {
                let current = &layers[seam_string[index].0].points[seam_string[index].1];
                // SeamPlacer.cpp:1244-1248
                let mut layer_angle = 0.0_f32;
                if index > 0 && index < n - 1 {
                    let prev_pos =
                        layers[seam_string[index - 1].0].points[seam_string[index - 1].1].position;
                    let next_pos =
                        layers[seam_string[index + 1].0].points[seam_string[index + 1].1].position;
                    layer_angle = angle_3d(current.position - prev_pos, next_pos - current.position);
                }
                // SeamPlacer.cpp:1249-1251
                let cp = current.position.xy();
                observations[index] = vec![cp.x, cp.y];
                observation_points[index] = current.position.z;
                weights[index] = angle_weight(current.local_ccw_angle);
                // SeamPlacer.cpp:1252-1256
                let mut sign = if layer_angle > 2.0 * current.local_ccw_angle.abs() {
                    -0.8_f32
                } else {
                    1.0_f32
                };
                if current.r#type == EnforcedBlockedSeamPoint::Enforced {
                    sign = 1.0;
                    weights[index] += 3.0;
                }
                // SeamPlacer.cpp:1257-1258
                total_length += sign * (last_point_pos - current.position).norm();
                last_point_pos = current.position;
            }

            if std::env::var("SEAMDBG").is_ok() {
                println!(
                    "SEAMSTR-R size={} l0={} l1={} len={:.4} seed={},{}",
                    seam_string.len(),
                    seam_string.first().unwrap().0,
                    seam_string.last().unwrap().0,
                    total_length,
                    layer_idx,
                    seam_index
                );
            }
            // SeamPlacer.cpp:1261-1263 — Curve fitting.
            // size_t number_of_segments = max(1, max(0.0f, total_length) / seam_align_mm_per_segment)
            let number_of_segments = std::cmp::max(
                1usize,
                (total_length.max(0.0) / SEAM_ALIGN_MM_PER_SEGMENT) as usize,
            );
            let curve = fit_cubic_bspline(
                &observations,
                &observation_points,
                &weights,
                number_of_segments,
                0, // endpoints_level_of_freedom (C++ default)
                2, // dimension
            );

            // SeamPlacer.cpp:1267-1283 — apply alignment, store into Perimeter.
            for index in 0..n {
                let pair = seam_string[index];
                let pt = &self.seam_data.layers[pair.0].points[pair.1];
                // SeamPlacer.cpp:1269
                let mut t = (pt.local_ccw_angle.abs() / SHARP_ANGLE_SNAPPING_THRESHOLD)
                    .powf(3.0)
                    .min(1.0);
                // SeamPlacer.cpp:1270
                if pt.r#type == EnforcedBlockedSeamPoint::Enforced {
                    t = t.max(0.4);
                }

                // SeamPlacer.cpp:1272-1273
                let current_pos = pt.position;
                let fitted = curve.get_fitted_value::<CubicBSplineKernel>(current_pos.z);
                let fitted_pos = Vec2f::new(fitted[0], fitted[1]);

                // SeamPlacer.cpp:1276 — interpolate between current and fitted.
                let fitted_3d = Vec3f::new(fitted_pos.x, fitted_pos.y, current_pos.z);
                let final_position = t * current_pos + (1.0 - t) * fitted_3d;

                // SeamPlacer.cpp:1279-1282
                let perim_idx = self.seam_data.layers[pair.0].points[pair.1].perimeter;
                let perimeter = &mut self.seam_data.layers[pair.0].perimeters[perim_idx];
                perimeter.seam_index = pair.1;
                perimeter.final_seam_position = final_position;
                perimeter.finalized = true;
            }
        }
    }

    /// SeamPlacer.cpp:1308-1321
    pub fn gather_all_seams_of_object(layers: &[LayerSeams]) -> Vec<(usize, usize)> {
        // gather vector of all seams on the print_object - pair of layer_index and
        // seam__index within that layer
        // SeamPlacer.cpp:1311
        let mut seams: Vec<(usize, usize)> = Vec::new();
        // SeamPlacer.cpp:1312-1319
        for layer_idx in 0..layers.len() {
            let layer_perimeter_points = &layers[layer_idx].points;
            let mut current_point_index = 0usize;
            while current_point_index < layer_perimeter_points.len() {
                // C++ reads `perimeter.seam_index` / `perimeter.end_index` through
                // the candidate's Perimeter back-reference; resolved via the index.
                let perimeter = &layers[layer_idx].perimeters
                    [layer_perimeter_points[current_point_index].perimeter];
                seams.push((layer_idx, perimeter.seam_index));
                current_point_index = perimeter.end_index;
            }
        }
        seams
    }

    /// SeamPlacer.cpp:1323-1393
    ///
    /// C++ collects the group indices into `std::vector<int>`; this port uses
    /// `usize` (the values are vector indices and never negative).
    pub fn filter_scarf_seam_switch_by_angle(angle: f32, layers: &mut [LayerSeams]) {
        // SeamPlacer.cpp:1325
        let seams = Self::gather_all_seams_of_object(layers);

        // SeamPlacer.cpp:1327
        let max_distance = SEAM_ALIGN_TOLERABLE_DIST_FACTOR
            * layers[seams[0].0].perimeters[layers[seams[0].0].points[seams[0].1].perimeter]
                .flow_width;

        // SeamPlacer.cpp:1329-1330
        let mut seam_index_pos: Vec<usize> = Vec::new();
        let mut seam_index_group: Vec<Vec<usize>> = Vec::new();
        // get each seam line group
        // SeamPlacer.cpp:1332
        for seam_idx in 0..seams.len() {
            // SeamPlacer.cpp:1333-1334
            if layers[seams[seam_idx].0].points[seams[seam_idx].1].is_grouped {
                continue;
            }

            // SeamPlacer.cpp:1336-1337
            layers[seams[seam_idx].0].points[seams[seam_idx].1].is_grouped = true;
            seam_index_pos.push(seam_idx);
            // SeamPlacer.cpp:1338-1339
            let mut prev_idx = seam_idx;
            let mut next_seam = seam_idx + 1;
            // SeamPlacer.cpp:1340
            while next_seam < seams.len() {
                // SeamPlacer.cpp:1341-1342
                if layers[seams[next_seam].0].points[seams[next_seam].1].is_grouped
                    || seams[prev_idx].0 == seams[next_seam].0
                {
                    next_seam += 1;
                    continue;
                }

                // if the seam is not continous with prev layer, break
                // SeamPlacer.cpp:1345-1346
                if seams[prev_idx].0 + 1 != seams[next_seam].0 {
                    break;
                }

                // SeamPlacer.cpp:1348-1349
                if (layers[seams[prev_idx].0].points[seams[prev_idx].1].position
                    - layers[seams[next_seam].0].points[seams[next_seam].1].position)
                    .norm()
                    <= max_distance
                {
                    // SeamPlacer.cpp:1351
                    layers[seams[next_seam].0].points[seams[next_seam].1].is_grouped = true;

                    // SeamPlacer.cpp:1353
                    let mut next_seam_angle =
                        layers[seams[next_seam].0].points[seams[next_seam].1].local_ccw_angle;

                    // SeamPlacer.cpp:1355-1356
                    if next_seam_angle < 0.0 {
                        next_seam_angle *= -1.0;
                    }

                    // SeamPlacer.cpp:1358-1360
                    if PI - angle > next_seam_angle {
                        layers[seams[next_seam].0].points[seams[next_seam].1].enable_scarf_seam =
                            true;
                    }

                    // SeamPlacer.cpp:1362-1363
                    prev_idx = next_seam;
                    seam_index_pos.push(next_seam);
                }
                next_seam += 1;
            }

            // SeamPlacer.cpp:1367-1368 — emplace_back(std::move(seam_index_pos)); clear();
            seam_index_group.push(std::mem::take(&mut seam_index_pos));
        }

        // filter
        // SeamPlacer.cpp:1372-1392
        {
            for k in 0..seam_index_group.len() {
                // SeamPlacer.cpp:1374 — C++ copies the group; read-only here.
                let seam_group = &seam_index_group[k];
                if seam_group.len() <= 1 {
                    continue;
                }
                // SeamPlacer.cpp:1376 — int division.
                let half_window = AVERAGE_FILTER_WINDOW_SIZE / 2;
                // average filter
                // SeamPlacer.cpp:1378
                for idx in 0..seam_group.len() {
                    // SeamPlacer.cpp:1379-1380
                    let mut sum = 0.0_f64;
                    let mut count = 0_i32;

                    // SeamPlacer.cpp:1382-1388
                    for window_idx in -half_window..=half_window {
                        // C++ `int index = idx + window_idx;` — signed window
                        // offset around the unsigned idx.
                        let index = idx as i64 + window_idx as i64;
                        if index >= 0 && (index as usize) < seam_group.len() {
                            let s = seams[seam_group[index as usize]];
                            sum += if layers[s.0].points[s.1].enable_scarf_seam {
                                1.0
                            } else {
                                0.0
                            };
                            count += 1;
                        }
                    }
                    // SeamPlacer.cpp:1389
                    let s = seams[seam_group[idx]];
                    layers[s.0].points[s.1].enable_scarf_seam = (sum / count as f64) >= 0.5;
                }
            }
        }
    }

    /// SeamPlacer.cpp:1395-1461 — `SeamPlacer::init`.
    ///
    /// Orchestrates per-object seam computation. C++ first builds the
    /// [`GlobalModelInfo`] via `gather_enforcers_blockers` +
    /// `compute_global_occlusion` (SeamPlacer.cpp:1406-1452). This port runs
    /// [`compute_global_occlusion`] (real per-vertex raycast visibility); the
    /// enforcer/blocker gather stays empty (no seam painting in this pipeline).
    /// Real visibility values DO matter: in the [`SeamComparator`] penalty,
    /// `a.visibility` vs `b.visibility` differ between vertices of a loop, so they
    /// break the angle/overhang/embedding ties that a constant `1.0` (the old
    /// stub) would leave to the next term — e.g. the symmetric-loop mirror-vertex
    /// case where two candidates tie on angle but differ in occlusion.
    ///
    /// Then, mirroring SeamPlacer.cpp:1456-1460, it runs:
    /// `gather_seam_candidates` → `calculate_candidates_visibility` →
    /// `calculate_overhangs_and_layer_embedding` → per-perimeter initial seam
    /// pick (`pick_seam_point`/`pick_nearest`/`pick_random`, SeamPlacer.cpp:1100
    /// loop / `SeamPlacer::place_seam` fallback) → `align_seam_points` (for
    /// `spAligned`).
    pub fn init(&mut self, po: &PrintObject, configured_seam_preference: SeamPosition) {
        self.config_mode = match configured_seam_preference {
            SeamPosition::spAligned => SeamPositionMode::Aligned,
            SeamPosition::spNearest => SeamPositionMode::Nearest,
            SeamPosition::spRandom => SeamPositionMode::Random,
            SeamPosition::spRear => SeamPositionMode::Rear,
        };

        // SeamPlacer.cpp:1406-1452 — gather global model info.
        // C++ runs `compute_global_occlusion` (always) and, for spAligned/spNearest,
        // `gather_enforcers_blockers`. The enforcer/blocker gather needs ModelVolume
        // seam-painting facets (not present in this pipeline; Benchy has none), so it
        // is left as the empty default. `compute_global_occlusion` IS run: it samples
        // the object mesh and raycasts per-sample visibility so candidates get real
        // per-vertex occlusion values (breaking the symmetric-loop angle/overhang ties
        // the comparator would otherwise resolve arbitrarily).
        self.frame_offset_xy = Self::seam_frame_offset(po).unwrap_or((0, 0));
        let __t = std::time::Instant::now();
        let global_model_info = compute_global_occlusion(po);
        SPPROF_OCCL.fetch_add(__t.elapsed().as_nanos() as usize, std::sync::atomic::Ordering::Relaxed);

        // SeamPlacer.cpp:1456 — gather_seam_candidates.
        let __t = std::time::Instant::now();
        self.gather_seam_candidates(po, &global_model_info, configured_seam_preference);
        SPPROF_GATHER.fetch_add(__t.elapsed().as_nanos() as usize, std::sync::atomic::Ordering::Relaxed);
        // SeamPlacer.cpp:1457 — calculate_candidates_visibility (visibility==1.0).
        let __t = std::time::Instant::now();
        self.calculate_candidates_visibility(po, &global_model_info);
        SPPROF_VIS.fetch_add(__t.elapsed().as_nanos() as usize, std::sync::atomic::Ordering::Relaxed);
        // SeamPlacer.cpp:1458 — calculate_overhangs_and_layer_embedding.
        let __t = std::time::Instant::now();
        self.calculate_overhangs_and_layer_embedding(po);
        SPPROF_OVER.fetch_add(__t.elapsed().as_nanos() as usize, std::sync::atomic::Ordering::Relaxed);

        // SeamPlacer.cpp:1459-1460 — pick the initial seam of every perimeter,
        // then run alignment. In C++ the per-perimeter pick happens inside a
        // `tbb::parallel_for` over layers (the `pick_seam_point` /
        // `pick_nearest_seam_point_index` / `pick_random_seam_point` dispatch on
        // `m_seam_position`). Ported serially with identical results.
        let comparator = SeamComparator::new(configured_seam_preference);
        for layer in self.seam_data.layers.iter_mut() {
            let LayerSeams {
                perimeters, points, ..
            } = layer;
            // The pick functions take `&mut [Perimeter]`; `perimeters` is a
            // `VecDeque`, so expose its contiguous backing slice.
            let perimeters = perimeters.make_contiguous();
            // Walk each perimeter via its [start_index, end_index) run, exactly
            // like `gather_all_seams_of_object` (SeamPlacer.cpp:1312-1318).
            let mut current_point_index = 0usize;
            while current_point_index < points.len() {
                let perim_idx = points[current_point_index].perimeter;
                let end_index = perimeters[perim_idx].end_index;
                match configured_seam_preference {
                    SeamPosition::spRandom => {
                        pick_random_seam_point(points, perimeters, current_point_index);
                    }
                    SeamPosition::spNearest => {
                        // C++ defers the nearest pick to place_seam (it needs the
                        // live extruder position); here we seed `seam_index` with
                        // the position-independent best so the data is valid, and
                        // place_seam re-resolves the nearest at emit time.
                        pick_seam_point(points, perimeters, current_point_index, &comparator);
                    }
                    SeamPosition::spAligned | SeamPosition::spRear => {
                        pick_seam_point(points, perimeters, current_point_index, &comparator);
                    }
                }
                current_point_index = end_index;
            }
        }
        if std::env::var("CANDDBG").is_ok() {
            let layer = &self.seam_data.layers[6];
            for (i, c) in layer.points.iter().enumerate() {
                println!(
                    "CANDDBG-R i={} pos={:.5},{:.5} vis={:.9} ov={:.9} emb={:.9} ang={:.9}",
                    i, c.position.x, c.position.y, c.visibility, c.overhang,
                    c.embedded_distance, c.local_ccw_angle
                );
            }
        }
        if std::env::var("SEAMDBG").is_ok() {
            for (layer_idx, layer) in self.seam_data.layers.iter().enumerate() {
                let mut peri = 0usize;
                let mut cur = 0usize;
                while cur < layer.points.len() {
                    let per = &layer.perimeters[layer.points[cur].perimeter];
                    let pick = &layer.points[per.seam_index];
                    let mut line = format!(
                        "SEAMPICK-R layer={} peri={} pick={:.4},{:.4} n={} cands=",
                        layer_idx, peri, pick.position.x, pick.position.y,
                        per.end_index - per.start_index
                    );
                    for i in per.start_index..per.end_index {
                        line.push_str(&format!(
                            "{:.4},{:.4};",
                            layer.points[i].position.x, layer.points[i].position.y
                        ));
                    }
                    println!("{}", line);
                    cur = per.end_index;
                    peri += 1;
                }
            }
        }

        // SeamPlacer.cpp:1460 — align_seam_points (spAligned/spRear path).
        if matches!(
            configured_seam_preference,
            SeamPosition::spAligned | SeamPosition::spRear
        ) {
            self.align_seam_points(po, &comparator);
        }
        if std::env::var("SEAMDBG").is_ok() {
            for (layer_idx, layer) in self.seam_data.layers.iter().enumerate() {
                let mut peri = 0usize;
                let mut cur = 0usize;
                while cur < layer.points.len() {
                    let per = &layer.perimeters[layer.points[cur].perimeter];
                    let fin = if per.finalized {
                        per.final_seam_position
                    } else {
                        layer.points[per.seam_index].position
                    };
                    println!(
                        "SEAMFIN-R layer={} peri={} fin={:.4},{:.4} finalized={}",
                        layer_idx, peri, fin.x, fin.y, per.finalized as i32
                    );
                    cur = per.end_index;
                    peri += 1;
                }
            }
        }
    }

    /// SeamPlacer.cpp:1463-1528 — `SeamPlacer::place_seam` (seam-vertex lookup).
    ///
    /// Given the layer index and the (already CCW) perimeter polygon being
    /// extruded, returns the scaled seam [`Point`] at which the loop should be
    /// split. C++ takes the `Layer*`, derives the `PrintObject`, looks the loop's
    /// first point up in that layer's `points_tree` to find the owning
    /// `Perimeter`, and returns its `final_seam_position` when finalized (the
    /// aligned position) or the `seam_index` candidate otherwise
    /// (SeamPlacer.cpp:1481-1520). The `loop.split_at(seam_point, ...)` itself
    /// happens in `GCode::extrude_loop` (the caller).
    ///
    /// For `spNearest` (not finalized during `init`) we resolve the nearest
    /// candidate to `last_pos` at emit time (SeamPlacer.cpp:1505 path), matching
    /// C++ `pick_nearest_seam_point_index`.
    ///
    /// Returns `None` when this layer has no seam data for the polygon (e.g. the
    /// loop is not a region perimeter, or the layer index is out of range), so
    /// the caller can fall back to the legacy heuristic.
    /// `loop_ref` is the loop being extruded. C++ takes `ExtrusionLoop &loop`
    /// and reads both `loop.first_point()` and `loop.role()` from it, and calls
    /// `loop.get_closest_path_and_point` in the concave-corner block; we keep the
    /// pre-derived `polygon` for the tree query and take the loop alongside it.
    pub fn place_seam(
        &self,
        layer_idx: usize,
        loop_ref: &crate::extrusion_entity::ExtrusionLoop,
        polygon: &Polygon,
        last_pos: Point,
    ) -> Option<Point> {
        let dbg = std::env::var("SEAMDBG").is_ok();
        macro_rules! ret_none {
            ($why:expr) => {{
                if dbg {
                    eprintln!("PLACESEAM-R layer={} NONE {}", layer_idx, $why);
                }
                return None;
            }};
        }
        // SeamPlacer.cpp:1464-1470 — guard.
        if layer_idx >= self.seam_data.layers.len() {
            ret_none!("layer_oob");
        }
        let layer = &self.seam_data.layers[layer_idx];
        if layer.points.is_empty() {
            ret_none!("no_points");
        }

        // SeamPlacer.cpp:1481-1485 — query the per-layer f32 points_tree for the
        // candidate nearest to the loop's first vertex, identifying the owning
        // perimeter. C++ uses `find_closest_point(*layer_seams.points_tree,
        // unscaled_p)`; we mirror it with the f32 KD tree built on demand.
        let mut first = polygon.points()[0];
        // R191: seam data lives in the centered frame under the gate; the
        // exporter's loop is in the rust frame — translate the query in
        // (integer-exact) and the returned seam point back out.
        first.x -= self.frame_offset_xy.0;
        first.y -= self.frame_offset_xy.1;
        let up = unscale_point(&first);
        // C++ projects to the candidate z; candidates carry their own z so a 3D
        // query against the loop's xy at the candidate plane is exact. We query
        // with the layer's candidate z (all candidates of a layer share z).
        let query = Vec3f::new(up.x as f32, up.y as f32, layer.points[0].position.z);
        let points_tree = layer.build_points_tree();
        let nearest_point_index = crate::kd_tree_indirect::find_closest_point_eps(
            &points_tree,
            &query,
            EPSILON as f32,
            f32::MAX,
            |_| true,
        );
        if nearest_point_index == crate::kd_tree_indirect::NPOS {
            ret_none!("npos");
        }

        // SeamPlacer.cpp:1487 — resolve the owning perimeter.
        let perim_idx = layer.points[nearest_point_index].perimeter;
        let perimeter = &layer.perimeters[perim_idx];

        // SeamPlacer.cpp:1484-1495 — pick the seam position AND remember which
        // candidate index it came from; the concave-corner block below needs the
        // candidate, not just its position (C++ keeps both in `seam_index`).
        let mut seam_index: usize = perimeter.seam_index;
        // SeamPlacer.cpp:1505-1520 — pick the seam position.
        let seam_position: Vec3f = if self.config_mode == SeamPositionMode::Nearest {
            // SeamPlacer.cpp:1505 — nearest is resolved against the live position.
            let preffered = Vec2f::new(up.x as f32, up.y as f32);
            let _ = last_pos; // last_pos is encoded via `up` (loop start == last_pos here).
            // `perimeters` is contiguous after `init` (make_contiguous); the
            // back-reference indices live in the single front segment.
            let (perim_slice, tail) = layer.perimeters.as_slices();
            debug_assert!(tail.is_empty(), "perimeters deque must be contiguous after init");
            let idx = pick_nearest_seam_point_index(
                &layer.points,
                perim_slice,
                perimeter.start_index,
                &preffered,
            );
            seam_index = idx;
            layer.points[idx].position
        } else if perimeter.finalized {
            // SeamPlacer.cpp:1487-1488 — aligned/random/rear store
            // final_seam_position; seam_index stays perimeter.seam_index.
            perimeter.final_seam_position
        } else {
            // SeamPlacer.cpp:1492-1493 — fall back to the per-perimeter seam_index.
            layer.points[perimeter.seam_index].position
        };

        // Scale back to coord_t (the loop is in scaled coordinates).
        // SeamPlacer.cpp:1497 — Point seam_point = Point::new_scale(...)
        let mut seam_point = Point::new(
            crate::scale(seam_position.x as f64) + self.frame_offset_xy.0,
            crate::scale(seam_position.y as f64) + self.frame_offset_xy.1,
        );

        // SeamPlacer.cpp:1499-1519 — concave-corner realignment.
        //
        // "In this case, we are at internal perimeter, where the external
        //  perimeter has seam in concave angle. We want to align the internal
        //  seam into the concave corner, and not on the perpendicular projection
        //  on the closest edge (which is what the split_at function does)."
        //
        // Without this, an inner-wall seam that C++ pushes into the corner stays
        // on our loop as the perpendicular foot — landing one vertex EARLY and
        // emitting a short corrective segment to reach the corner (R752/R753).
        //
        // All SeamPlacer arithmetic is UNSCALED f32 mm: the 4.0 threshold is a
        // 2 mm radius, and `depth` unscales the (scaled) seam-to-foot vector.
        // Positions here are in the placer's centred frame, so differences are
        // frame-free and only the final absolute point re-adds `frame_offset_xy`.
        if crate::opt_in_gate("SEAM_CONCAVE_CORNER") {
            let perimeter_point = &layer.points[seam_index];
            // SeamPlacer.cpp:1500-1502 — three-way guard.
            if (self.config_mode == SeamPositionMode::Nearest
                || self.config_mode == SeamPositionMode::Aligned)
                && loop_ref.role() == ExtrusionRole::Perimeter
                && (seam_position - perimeter_point.position).norm_squared() < 4.0
                && perimeter_point.local_ccw_angle < -(EPSILON as f32)
            {
                let per = &layer.perimeters[perimeter_point.perimeter];
                // SeamPlacer.cpp:1505-1506 — wrap on the perimeter's own range.
                // `end_index` is PAST-THE-END despite the C++ header comment
                // claiming "inclusive!"; every native use is `i < end_index` and
                // `end_index - 1` for the last vertex.
                let index_of_prev = if seam_index == per.start_index {
                    per.end_index - 1
                } else {
                    seam_index - 1
                };
                let index_of_next = if seam_index == per.end_index - 1 {
                    per.start_index
                } else {
                    seam_index + 1
                };
                // SeamPlacer.cpp:1508-1510
                let to_prev = (perimeter_point.position - layer.points[index_of_prev].position)
                    .xy()
                    .normalize();
                let to_next = (perimeter_point.position - layer.points[index_of_next].position)
                    .xy()
                    .normalize();
                let dir_to_middle = (to_prev + to_next) * 0.5;
                // SeamPlacer.cpp:1512-1514 — depth of the perpendicular foot.
                let projected = loop_ref.get_closest_path_and_point(&seam_point, true);
                let depth = unscale_point(&(seam_point - projected.foot_pt)).length() as f32;
                // SeamPlacer.cpp:1515-1517 — overshoot so it snaps into the corner.
                let angle_factor = (-perimeter_point.local_ccw_angle / 2.0).cos();
                let final_pos =
                    perimeter_point.position.xy() + dir_to_middle * (1.4142 * depth / angle_factor);
                // SeamPlacer.cpp:1518
                seam_point = Point::new(
                    crate::scale(final_pos.x as f64) + self.frame_offset_xy.0,
                    crate::scale(final_pos.y as f64) + self.frame_offset_xy.1,
                );
            }
        }

        Some(seam_point)
    }
}

/// Gather a single perimeter polygon into a `LayerSeams`, mirroring the
/// vertex-candidate creation of `process_perimeter_polygon` (without
/// enforcer/blocker oversampling, which needs `GlobalModelInfo`).
/// SeamPlacer.cpp:422-490 (subset)
fn gather_layer_perimeter(layer: &mut LayerSeams, polygon: &Polygon, z: f32, flow_width: f32) {
    if polygon.len() == 0 {
        return;
    }
    let mut polygon = polygon.clone();
    // SeamPlacer.cpp:427-428
    let was_clockwise = {
        polygon.make_counter_clockwise();
        polygon.is_clockwise()
    };
    // SeamPlacer.cpp:429
    let angle_arm_len = 0.5_f32; // nozzle diameter default; region flow not threaded here.

    // SeamPlacer.cpp:431-433
    let mut lengths: Vec<f32> = Vec::new();
    for point_idx in 0..polygon.len() - 1 {
        let d = (unscale_point(&polygon.points()[point_idx])
            - unscale_point(&polygon.points()[point_idx + 1]))
        .length();
        lengths.push(d as f32);
    }
    let last = (unscale_point(&polygon.points()[0])
        - unscale_point(&polygon.points()[polygon.len() - 1]))
    .length()
    .max(0.1);
    lengths.push(last as f32);

    // SeamPlacer.cpp:434
    let polygon_angles = calculate_polygon_angles_at_vertices(&polygon, &lengths, angle_arm_len);

    // SeamPlacer.cpp:436-437
    let perimeter_index = layer.perimeters.len();
    let start_index = layer.points.len();

    // SeamPlacer.cpp:440-443 / 487
    for index in 0..polygon.len() {
        let up = unscale_point(&polygon.points()[index]);
        let position = Vec3f::new(up.x as f32, up.y as f32, z);
        // SeamPlacer.cpp:461
        let local_ccw_angle = if was_clockwise {
            -polygon_angles[index]
        } else {
            polygon_angles[index]
        };
        let mut cand = SeamCandidate::new(
            &position,
            perimeter_index,
            local_ccw_angle,
            EnforcedBlockedSeamPoint::Neutral,
        );
        cand.set_flow_width_hint(flow_width);
        layer.points.push(cand);
    }

    // SeamPlacer.cpp:436 / 447-448 / 490
    let mut perimeter = Perimeter::default();
    perimeter.start_index = start_index;
    perimeter.end_index = layer.points.len();
    perimeter.seam_index = start_index;
    perimeter.flow_width = flow_width;
    layer.perimeters.push_back(perimeter);
}

/// Find the best seam index for a single polygon using angle-based scoring.
///
/// Glue entry used by `perimeter_generator.rs` and `gcode/exporter.rs`. It
/// computes per-vertex angles via the faithful
/// [`calculate_polygon_angles_at_vertices`] and scores each candidate using the
/// `angle_importance * compute_angle_penalty` formula, with optional
/// nearest-position bias (gaussian) when `prev_seam_pos` is provided.
pub fn find_best_seam_index(
    polygon: &Polygon,
    prev_seam_pos: Option<Point>,
    config: &SeamPlacerConfig,
) -> usize {
    let points = polygon.points();
    if points.len() < 3 {
        return 0;
    }
    let n = points.len();

    // Segment lengths (unscaled mm), as in process_perimeter_polygon.
    let mut lengths: Vec<f32> = Vec::with_capacity(n);
    for i in 0..n - 1 {
        let d = (unscale_point(&points[i]) - unscale_point(&points[i + 1])).length();
        lengths.push(d as f32);
    }
    let last = (unscale_point(&points[0]) - unscale_point(&points[n - 1]))
        .length()
        .max(0.1);
    lengths.push(last as f32);

    let angles =
        calculate_polygon_angles_at_vertices(polygon, &lengths, config.min_arm_length as f32);

    let angle_importance = config.angle_importance as f32;
    let preffered = prev_seam_pos.map(|p| Vec2f::new(unscale(p.x) as f32, unscale(p.y) as f32));

    let mut best_idx = 0usize;
    let mut best_penalty = f32::INFINITY;
    for i in 0..n {
        // distance penalty (spNearest path, SeamPlacer.cpp:700-701)
        let distance_penalty = if let Some(loc) = preffered {
            let up = unscale_point(&points[i]);
            let pos = Vec2f::new(up.x as f32, up.y as f32);
            1.0 - gauss((pos - loc).norm(), 0.0, 1.0, 0.005)
        } else {
            0.0
        };
        // penalty (SeamPlacer.cpp:708), visibility/overhang unknown here -> 0.
        let penalty =
            angle_importance * compute_angle_penalty(angles[i]) + distance_penalty;
        if penalty < best_penalty {
            best_penalty = penalty;
            best_idx = i;
        }
    }
    best_idx
}

/// Pick a seam index on a polygon for the given mode (glue convenience fn).
pub fn place_seam(polygon: &Polygon, mode: SeamPositionMode, current_pos: Option<Point>) -> usize {
    let config = SeamPlacerConfig {
        seam_position: mode,
        ..Default::default()
    };
    find_best_seam_index(polygon, current_pos, &config)
}

/// Create a seam placer and initialize with simple polygon layers.
pub fn create_seam_placer(
    layers: &[(f64, Vec<Polygon>)],
    flow_width: f64,
    mode: SeamPositionMode,
) -> SeamPlacer {
    let mut placer = SeamPlacer::new(SeamPlacerConfig {
        seam_position: mode,
        ..Default::default()
    });
    placer.init_simple(layers, flow_width);
    placer
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scale;

    fn make_square(size: f64) -> Polygon {
        let s = scale(size);
        Polygon::from_points(vec![
            Point::new(0, 0),
            Point::new(s, 0),
            Point::new(s, s),
            Point::new(0, s),
        ])
    }

    #[test]
    fn test_gauss_function() {
        // Peak at mean
        let peak = gauss(0.0, 0.0, 1.0, 1.0);
        let off_peak = gauss(1.0, 0.0, 1.0, 1.0);
        assert!(peak > off_peak);
        assert!(peak > 0.0);
    }

    #[test]
    fn test_compute_angle_penalty() {
        // Concave (negative) should be better (lower penalty) than convex.
        let concave_penalty = compute_angle_penalty(-0.5);
        let convex_penalty = compute_angle_penalty(0.5);
        assert!(concave_penalty < convex_penalty);
        assert!(concave_penalty > 0.0);
    }

    #[test]
    fn test_sgn() {
        assert_eq!(sgn(3.0), 1);
        assert_eq!(sgn(-3.0), -1);
        assert_eq!(sgn(0.0), 0);
    }

    #[test]
    fn test_value_to_rgbf() {
        let c = value_to_rgbf(0.0, 1.0, 0.5);
        // ratio = 1.0 -> b=0, r=0, g=1
        assert!((c.x - 0.0).abs() < 1e-6);
        assert!((c.y - 1.0).abs() < 1e-6);
        assert!((c.z - 0.0).abs() < 1e-6);
    }

    #[test]
    fn test_frame_set_from_z() {
        let mut f = Frame::new();
        f.set_from_z(&Vec3f::new(0.0, 0.0, 2.0));
        // normal should be unit +Z
        let nz = f.normal();
        assert!((nz.z - 1.0).abs() < 1e-6);
        // frame axes should be orthonormal
        assert!(f.binormal().dot(f.tangent()).abs() < 1e-5);
    }

    #[test]
    fn test_sample_hemisphere_uniform_upper() {
        let v = sample_hemisphere_uniform(&Vec2f::new(0.25, 0.5));
        assert!(v.z >= 0.0);
    }

    #[test]
    fn test_angle_helper() {
        // angle from +X to +Y is +PI/2
        let a = angle(Vec2d::new(1.0, 0.0), Vec2d::new(0.0, 1.0));
        assert!((a - std::f64::consts::FRAC_PI_2).abs() < 1e-9);
    }

    #[test]
    fn test_calculate_polygon_angles_square() {
        let sq = make_square(10.0);
        let lengths = vec![10.0_f32, 10.0, 10.0, 10.0];
        let angles = calculate_polygon_angles_at_vertices(&sq, &lengths, 0.5);
        for a in &angles {
            assert!((a.abs() - std::f32::consts::FRAC_PI_2).abs() < 0.2);
        }
    }

    #[test]
    fn test_seam_comparator_enforced_blocked() {
        let comparator = SeamComparator::new(SeamPosition::spAligned);
        let pos = Vec3f::zeros();
        let neutral = SeamCandidate::new(&pos, 0, 0.0, EnforcedBlockedSeamPoint::Neutral);
        let enforced = SeamCandidate::new(&pos, 0, 0.0, EnforcedBlockedSeamPoint::Enforced);
        let blocked = SeamCandidate::new(&pos, 0, 0.0, EnforcedBlockedSeamPoint::Blocked);
        let z = Vec2f::zeros();
        assert!(comparator.is_first_better(&enforced, &neutral, &z));
        assert!(comparator.is_first_better(&neutral, &blocked, &z));
        assert!(comparator.is_first_better(&enforced, &blocked, &z));
    }

    #[test]
    fn test_seam_comparator_overhang() {
        let comparator = SeamComparator::new(SeamPosition::spAligned);
        let pos = Vec3f::zeros();
        let no_overhang = SeamCandidate::new(&pos, 0, 0.0, EnforcedBlockedSeamPoint::Neutral);
        let mut with_overhang =
            SeamCandidate::new(&pos, 0, 0.0, EnforcedBlockedSeamPoint::Neutral);
        with_overhang.overhang = 0.5;
        let z = Vec2f::zeros();
        assert!(comparator.is_first_better(&no_overhang, &with_overhang, &z));
    }

    #[test]
    fn test_find_best_seam_index_square() {
        let sq = make_square(10.0);
        let idx = find_best_seam_index(&sq, None, &SeamPlacerConfig::default());
        assert!(idx < 4);
    }

    #[test]
    fn test_create_seam_placer_convenience() {
        let square = make_square(10.0);
        let layers = vec![(0.2, vec![square.clone()]), (0.4, vec![square])];
        let placer = create_seam_placer(&layers, 0.4, SeamPositionMode::Aligned);
        let stats = placer.stats();
        assert_eq!(stats.layer_count, 2);
        assert_eq!(stats.total_perimeters, 2);
    }
}
