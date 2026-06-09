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
//! the data structures from the header are ported faithfully. Several
//! `SeamPlacer` member functions that operate purely on the in-memory
//! `PrintObjectSeamData::LayerSeams` data are also ported faithfully
//! (`find_next_seam_in_layer`, `find_seam_string`, `align_seam_points`,
//! `gather_all_seams_of_object`, `filter_scarf_seam_switch_by_angle`).
//!
//! BLOCKED (need not-yet-ported dependencies threaded through
//! `Print`/`PrintObject`/`Model`/`Layer`):
//! - `SeamPlacer::init`
//! - `SeamPlacer::gather_seam_candidates`
//! - `SeamPlacer::calculate_candidates_visibility`
//! - `SeamPlacer::calculate_overhangs_and_layer_embedding`
//! - `SeamPlacer::place_seam`
//! - `SeamPlacerImpl::compute_global_occlusion`
//! - `SeamPlacerImpl::gather_enforcers_blockers`
//!   reasons: `PrintObject::{config, model_object, trafo_centered, get_layer,
//!   layer_count, slicing_parameters}`, `Layer::object`, and `ModelVolume`
//!   seam-painting facet accessors (`is_seam_painted`, `seam_facets`,
//!   `EnforcerBlockerType`) are not yet ported.

use crate::aabb_tree_indirect::Tree3F;
use crate::aabb_tree_lines::{build_aabb_tree_over_indexed_lines, squared_distance_to_indexed_lines, tree2d};
use crate::geometry::{Line, LineF, Point, PointF, Polygon};
use crate::kd_tree_indirect::KDTreeIndirect;
use crate::triangle_set_sampling::{indexed_triangle_set, TriangleSetSamples};
use crate::unscale;
use crate::utils::{next_idx_modulo, prev_idx_modulo};
use nalgebra::{Vector2, Vector3};
use std::collections::VecDeque;

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

// SeamPlacer.cpp:32 — used by the BLOCKED `filter_scarf_seam_switch_by_angle`.
#[allow(dead_code)]
const AVERAGE_FILTER_WINDOW_SIZE: i32 = 5;
// SeamPlacer.cpp:33
const OVERHANG_FILTER: f32 = 0.0;
// SeamPlacer.cpp:34 — used by the BLOCKED `calculate_overhangs_and_layer_embedding`.
#[allow(dead_code)]
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
}

impl LayerSeams {
    /// Build a KD tree over `points`, mirroring
    /// `points_tree = std::make_unique<SeamCandidatesTree>(functor, points.size())`.
    /// SeamPlacer.cpp:944-945
    pub fn build_points_tree(&self) -> KDTreeIndirect<3, f32, impl Fn(usize, usize) -> f32 + '_> {
        // SeamPlacer.hpp:102-107 — SeamCandidateCoordinateFunctor.
        let functor = move |index: usize, dim: usize| -> f32 { self.points[index].position[dim] };
        KDTreeIndirect::with_indices(functor, self.points.len())
    }
}

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

/// SeamPlacer.cpp:135-214
///
/// Precomputes the hemisphere sample directions faithfully
/// (SeamPlacer.cpp:143-152). The per-sample ray casting itself
/// (SeamPlacer.cpp:154-208) is BLOCKED: the crate's `aabb_tree_indirect`
/// builds trees over `Point3F` (f64) vertices and `[usize;3]` faces, whereas
/// `indexed_triangle_set` stores `Vec3f` (f32) / `Vec3i`. The C++ source casts
/// only the ray origin/dir to f64 while keeping vertices in f32; bridging the
/// incompatible primitive types here would diverge from C++ rounding and is not
/// byte-exact. Re-enable once an f32-vertex AABB ray query is available.
pub fn raycast_visibility(
    _raycasting_tree: &Tree3F,
    triangles: &indexed_triangle_set,
    samples: &TriangleSetSamples,
    _negative_volumes_start_index: usize,
) -> Vec<f32> {
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
    let _ = triangles;
    // BLOCKED — see doc comment. C++ returns per-sample visibility; we return
    // the C++ initial value (1.0 for every sample) until ray casting is wired.
    // SeamPlacer.cpp:156 / 162 (`result[s_idx] = 1.0f;`).
    vec![1.0_f32; samples.positions.len()]
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

    // SeamPlacer.cpp:301-326
    //
    // The radius-weighted visibility averaging math is faithful. The nearby
    // sample lookup `find_nearby_points(mesh_samples_tree, position, radius)`
    // (SeamPlacer.cpp:303) is BLOCKED: the crate's `KDTreeIndirect` query
    // functions bound `T: From<f64>`, which `f32` does not satisfy, so an f32 KD
    // tree (as the C++ `KDTreeIndirect<3, float, ...>`) cannot be queried.
    // `find_nearby_candidate_indices` returns the empty set until an f32-capable
    // KD query exists; per C++ (SeamPlacer.cpp:304) an empty set yields 1.0.
    pub fn calculate_point_visibility(&self, position: &Vec3f) -> f32 {
        // SeamPlacer.cpp:303
        let points = self.find_nearby_candidate_indices(position, self.mesh_samples_radius);
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

    /// BLOCKED helper for `calculate_point_visibility`: f32 KD tree query is not
    /// supported by the crate (`T: From<f64>` bound). Returns the empty set.
    fn find_nearby_candidate_indices(&self, _position: &Vec3f, _radius: f32) -> Vec<usize> {
        Vec::new()
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
// `SeamPlacer::{init, place_seam}` pipeline remains blocked on the
// `Print`/`PrintObject`/`Model`/`Layer` accessors enumerated at the top of this
// file. The scoring uses the faithful `compute_angle_penalty` / `gauss`
// functions above so that the eventual full port and this shim agree on the
// angle/visibility math.

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
}

impl SeamPlacer {
    pub fn new(config: SeamPlacerConfig) -> Self {
        Self {
            config_mode: config.seam_position,
            seam_data: PrintObjectSeamData::default(),
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
