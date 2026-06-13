//! Build volume collision detection and validation
//!
//! C++ Reference:
//! - BuildVolume.hpp (class definitions and enums)
//! - BuildVolume.cpp (constructor and collision detection methods)
//!
//! 1:1 faithful port of `src/libslic3r/BuildVolume.cpp` + `BuildVolume.hpp`.
//!
//! coord_t -> i64, coordf_t -> f64. Reuses crate primitives:
//! - `geometry::Circle` (= C++ `Geometry::Circled`)
//! - `geometry::circle_ransac`, `geometry::smallest_enclosing_circle_welzl`
//! - `geometry::convex_hull_points` (= `Geometry::convex_hull`)
//! - `geometry::decompose_convex_polygon_top_bottom`, `geometry::inside_convex_polygon`
//! - `clipper_utils::offset_polygons` (= `expand`)
//! - `triangle_set_sampling::indexed_triangle_set`, `geometry::Transform3D` (= `Transform3f`)
//!
//! Blocked symbols (see notes at the bottom of the file):
//! - `all_paths_inside` — depends on `GCodeProcessorResult::moves` / `MoveVertex`
//!   (the crate's `GCodeProcessorResult` does not expose the per-move vertex list).

use crate::bounding_box::{BoundingBoxf, BoundingBoxf3};
use crate::geometry::Transform3D;
use crate::geometry::{
    convex_hull_points, decompose_convex_polygon_top_bottom, inside_convex_polygon, to_polygons,
    BoundingBox, Circle, Point, Point3F, Polygon, Vec2d, Vec3d,
};
use crate::triangle_set_sampling::{indexed_triangle_set, Vec3f, Vec3i};
use crate::triangle_mesh::its_make_cube;
use crate::clipper_utils::{self, OffsetJoinType};
use crate::{scale, unscale, SCALING_FACTOR};

/// `scaled<double>(v)`: scale a floating point value but keep the result as a
/// double (no truncation to integer), mirroring C++ `scaled<double>`.
/// libslic3r.h: `template<class T> coordf_t scaled(coordf_t v) { return v / SCALING_FACTOR; }`
/// Here we follow the crate scaling convention: `scaled(v) = v * SCALING_FACTOR`.
#[inline]
fn scaled_f64(v: f64) -> f64 {
    v * SCALING_FACTOR
}

/// `unscaled<double>(v)`: unscale a double-precision coordinate.
/// libslic3r.h: `template<class T> coordf_t unscaled(coordf_t v) { return v * SCALING_FACTOR; }`
/// Crate convention: `unscaled(v) = v / SCALING_FACTOR`.
#[inline]
fn unscaled_f64(v: f64) -> f64 {
    v / SCALING_FACTOR
}

/// `sqr(x)` — libslic3r.h
#[inline]
fn sqr(x: f64) -> f64 {
    x * x
}

/// Epsilon for floating-point comparisons (from libslic3r.h)
/// libslic3r.h: `static constexpr double EPSILON = 1e-4;`
const EPSILON: f64 = 1e-4;

/// Scaled epsilon for integer coordinate comparisons (kept as f64 since it is
/// only ever used inside `sqr(SCALED_EPSILON)` floating-point comparisons).
/// libslic3r.h: `#define SCALED_EPSILON scaled<double>(EPSILON)`
const SCALED_EPSILON: f64 = EPSILON * SCALING_FACTOR;

/// Epsilon for scene collision tests (plater UI)
/// BuildVolume.hpp:107
/// C++: static constexpr const double SceneEpsilon = EPSILON;
pub const SCENE_EPSILON: f64 = EPSILON;

/// Epsilon for bed collision tests (G-code paths)
/// BuildVolume.hpp:118
/// C++: static constexpr const double BedEpsilon = 3. * EPSILON;
pub const BED_EPSILON: f64 = 3.0 * EPSILON;

/// Build volume type classification
/// BuildVolume.hpp:20-32
/// C++: enum class Type : char
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Type {
    /// Not set yet or undefined
    /// BuildVolume.hpp:23
    /// C++: Invalid = -1,
    Invalid,
    /// Rectangular print bed (most common, cheap to work with)
    /// BuildVolume.hpp:25
    /// C++: Rectangle,
    Rectangle,
    /// Circular print bed (common on deltas, cheap to work with)
    /// BuildVolume.hpp:27
    /// C++: Circle,
    Circle,
    /// Convex print bed (complex to process)
    /// BuildVolume.hpp:29
    /// C++: Convex,
    Convex,
    /// Some non-convex shape
    /// BuildVolume.hpp:31
    /// C++: Custom
    Custom,
}

impl Type {
    /// Numeric value matching C++ `enum class Type : char` (Invalid = -1, ...).
    /// Used for `m_shared_volume.type = static_cast<int>(m_type)`.
    /// BuildVolume.hpp:23-31
    fn to_int(self) -> i32 {
        match self {
            Type::Invalid => -1,
            Type::Rectangle => 0,
            Type::Circle => 1,
            Type::Convex => 2,
            Type::Custom => 3,
        }
    }
}

/// Object collision state with build volume
/// BuildVolume.hpp:90-103
/// C++: enum class ObjectState : unsigned char
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ObjectState {
    /// Inside the build volume, thus printable
    /// BuildVolume.hpp:93
    /// C++: Inside,
    Inside,
    /// Colliding with build volume boundary, not printable (error shown)
    /// BuildVolume.hpp:95
    /// C++: Colliding,
    Colliding,
    /// Outside of build volume (object ignored, no error)
    /// BuildVolume.hpp:97
    /// C++: Outside,
    Outside,
    /// Completely below the print bed
    /// BuildVolume.hpp:100
    /// C++: Below,
    Below,
    /// In limited area (extruder-specific constraint)
    /// BuildVolume.hpp:102
    /// C++: Limited
    Limited,
}

/// Per-extruder build volume (for multi-extruder limited areas)
/// BuildVolume.hpp:34-40
/// C++: struct BuildExtruderVolume
#[derive(Debug, Clone)]
pub struct BuildExtruderVolume {
    /// BuildVolume.hpp:35
    /// C++: bool same_with_bed{false};
    pub same_with_bed: bool,
    /// BuildVolume.hpp:36
    /// C++: Type type{Type::Invalid};
    pub volume_type: Type,
    /// BuildVolume.hpp:37
    /// C++: BoundingBox bbox;
    pub bbox: BoundingBox,
    /// BuildVolume.hpp:38
    /// C++: BoundingBoxf3 bboxf;
    pub bboxf: BoundingBoxf3,
    /// BuildVolume.hpp:39
    /// C++: Geometry::Circled circle;
    pub circle: Circle,
}

impl BuildExtruderVolume {
    /// Create a new empty extruder volume matching C++ default member initializers.
    /// BuildVolume.hpp:34-40
    pub fn new() -> Self {
        Self {
            same_with_bed: false,
            volume_type: Type::Invalid,
            bbox: BoundingBox::new(),
            bboxf: BoundingBoxf3::new(),
            // Geometry::Circled default-constructs to { Vec2d::Zero(), 0 }.
            circle: Circle::new(Vec2d::new(0.0, 0.0), 0.0),
        }
    }
}

impl Default for BuildExtruderVolume {
    fn default() -> Self {
        Self::new()
    }
}

/// Shared volume for rendering (simplified representation)
/// BuildVolume.hpp:42-54
/// C++: struct BuildSharedVolume
#[derive(Debug, Clone, Copy)]
pub struct BuildSharedVolume {
    /// Bed shape type (see Bed3D::EShapeType)
    /// BuildVolume.hpp:45
    /// C++: int type{ 0 };
    pub volume_type: i32,
    /// Rectangle: [min.x, min.y, max.x, max.y]; Circle: [center.x, center.y, -, radius]
    /// BuildVolume.hpp:51
    /// C++: std::array<float, 4> data;
    pub data: [f32; 4],
    /// Z range: [min_z, max_z]
    /// BuildVolume.hpp:53
    /// C++: std::array<float, 2> zs;
    pub zs: [f32; 2],
}

impl BuildSharedVolume {
    /// Create a new empty shared volume
    pub fn new() -> Self {
        Self {
            volume_type: 0,
            data: [0.0; 4],
            zs: [0.0; 2],
        }
    }
}

impl Default for BuildSharedVolume {
    fn default() -> Self {
        Self::new()
    }
}

/// Build volume for collision detection
/// BuildVolume.hpp:17-162
/// C++: class BuildVolume
#[derive(Debug, Clone)]
pub struct BuildVolume {
    /// Source bed shape (unscaled coordinates)
    /// BuildVolume.hpp:134
    /// C++: std::vector<Vec2d> m_bed_shape;
    bed_shape: Vec<Vec2d>,
    /// Per-extruder shapes (unscaled coordinates)
    /// BuildVolume.hpp:136
    /// C++: std::vector<std::vector<Vec2d>> m_extruder_shapes;
    extruder_shapes: Vec<Vec<Vec2d>>,
    /// Per-extruder volumes
    /// BuildVolume.hpp:137
    /// C++: std::vector<BuildExtruderVolume> m_extruder_volumes;
    extruder_volumes: Vec<BuildExtruderVolume>,
    /// Shared volume for rendering
    /// BuildVolume.hpp:138
    /// C++: BuildSharedVolume m_shared_volume;
    shared_volume: BuildSharedVolume,
    /// Maximum print height (unscaled)
    /// BuildVolume.hpp:140
    /// C++: double m_max_print_height { 0.f };
    max_print_height: f64,
    /// Per-extruder printable heights
    /// BuildVolume.hpp:141
    /// C++: std::vector<double> m_extruder_printable_height;
    extruder_printable_height: Vec<f64>,
    /// Derived volume type
    /// BuildVolume.hpp:144
    /// C++: Type m_type { Type::Invalid };
    volume_type: Type,
    /// Bed geometry (scaled coordinates)
    /// BuildVolume.hpp:146
    /// C++: Polygon m_polygon;
    polygon: Polygon,
    /// Snug bounding box around polygon (scaled)
    /// BuildVolume.hpp:148
    /// C++: BoundingBox m_bbox;
    bbox: BoundingBox,
    /// 3D bounding volume (unscaled)
    /// BuildVolume.hpp:150
    /// C++: BoundingBoxf3 m_bboxf;
    bboxf: BoundingBoxf3,
    /// Area of polygon (scaled)
    /// BuildVolume.hpp:152
    /// C++: double m_area { 0. };
    area: f64,
    /// Convex hull of polygon (scaled)
    /// BuildVolume.hpp:154
    /// C++: Polygon m_convex_hull;
    convex_hull: Polygon,
    /// Convex hull decomposition for scene tests
    /// BuildVolume.hpp:157
    /// C++: std::pair<std::vector<Vec2d>, std::vector<Vec2d>> m_top_bottom_convex_hull_decomposition_scene;
    top_bottom_convex_hull_decomposition_scene: (Vec<Vec2d>, Vec<Vec2d>),
    /// Convex hull decomposition for bed tests
    /// BuildVolume.hpp:159
    /// C++: std::pair<std::vector<Vec2d>, std::vector<Vec2d>> m_top_bottom_convex_hull_decomposition_bed;
    top_bottom_convex_hull_decomposition_bed: (Vec<Vec2d>, Vec<Vec2d>),
    /// Smallest enclosing circle (scaled)
    /// BuildVolume.hpp:161
    /// C++: Geometry::Circled m_circle { Vec2d::Zero(), 0 };
    circle: Circle,
}

impl BuildVolume {
    /// Create an uninitialized build volume
    /// BuildVolume.hpp:57
    /// C++: BuildVolume() {}
    pub fn new() -> Self {
        Self {
            bed_shape: Vec::new(),
            extruder_shapes: Vec::new(),
            extruder_volumes: Vec::new(),
            shared_volume: BuildSharedVolume::new(),
            max_print_height: 0.0,
            extruder_printable_height: Vec::new(),
            volume_type: Type::Invalid,
            polygon: Polygon::new(),
            bbox: BoundingBox::new(),
            bboxf: BoundingBoxf3::new(),
            area: 0.0,
            convex_hull: Polygon::new(),
            top_bottom_convex_hull_decomposition_scene: (Vec::new(), Vec::new()),
            top_bottom_convex_hull_decomposition_bed: (Vec::new(), Vec::new()),
            circle: Circle::new(Vec2d::new(0.0, 0.0), 0.0),
        }
    }

    /// Initialize from printable area, height, and extruder areas
    /// BuildVolume.cpp:12-176
    /// C++: BuildVolume::BuildVolume(const std::vector<Vec2d> &printable_area, const double printable_height,
    /// C++:     const std::vector<std::vector<Vec2d>> &extruder_areas, const std::vector<double>& extruder_printable_heights)
    pub fn new_from_config(
        printable_area: Vec<Vec2d>,
        printable_height: f64,
        extruder_areas: Vec<Vec<Vec2d>>,
        extruder_printable_heights: Vec<f64>,
    ) -> Self {
        // BuildVolume.cpp:13 — member init list.
        let mut bv = Self::new();
        bv.bed_shape = printable_area.clone();
        bv.max_print_height = printable_height;
        bv.extruder_shapes = extruder_areas.clone();
        bv.extruder_printable_height = extruder_printable_heights.clone();

        // BuildVolume.cpp:15
        // C++: assert(printable_height >= 0);
        assert!(printable_height >= 0.0);
        // BuildVolume.cpp:16
        // C++: //assert(extruder_printable_heights.size() == extruder_areas.size());

        // BuildVolume.cpp:18
        // C++: m_polygon = Polygon::new_scale(printable_area);
        bv.polygon = Polygon::from_points(
            printable_area
                .iter()
                .map(|p| Point::new(scale(p.x), scale(p.y)))
                .collect(),
        );

        // Calcuate various metrics of the input polygon.
        // BuildVolume.cpp:21
        // C++: m_convex_hull = Geometry::convex_hull(m_polygon.points);
        bv.convex_hull = convex_hull_points(bv.polygon.points.clone());
        // BuildVolume.cpp:22
        // C++: m_bbox = get_extents(m_convex_hull);
        bv.bbox = bv.convex_hull.bounding_box();
        // BuildVolume.cpp:23
        // C++: m_area = m_polygon.area();
        bv.area = bv.polygon.area();

        // BuildVolume.cpp:25
        // C++: BoundingBoxf bboxf = get_extents(printable_area);
        let bboxf: BoundingBoxf = BoundingBoxf::new_from_points_slice(&printable_area);
        // BuildVolume.cpp:26
        // C++: m_bboxf = BoundingBoxf3{ to_3d(bboxf.min, 0.), to_3d(bboxf.max, printable_height) };
        bv.bboxf = BoundingBoxf3::new_from_points(
            Vec3d::new(bboxf.min.x, bboxf.min.y, 0.0),
            Vec3d::new(bboxf.max.x, bboxf.max.y, printable_height),
        );

        // BuildVolume.cpp:28
        // C++: if (printable_area.size() >= 4 && std::abs((m_area - double(m_bbox.size().x()) * double(m_bbox.size().y()))) < sqr(SCALED_EPSILON)) {
        if printable_area.len() >= 4
            && (bv.area - (bv.bbox.size().x() as f64) * (bv.bbox.size().y() as f64)).abs()
                < sqr(SCALED_EPSILON)
        {
            // Square print bed, use the bounding box for collision detection.
            // BuildVolume.cpp:30
            // C++: m_type = Type::Rectangle;
            bv.volume_type = Type::Rectangle;
            // BuildVolume.cpp:31
            // C++: m_circle.center = 0.5 * (m_bbox.min.cast<double>() + m_bbox.max.cast<double>());
            bv.circle.center = Vec2d::new(
                0.5 * ((bv.bbox.min.x() as f64) + (bv.bbox.max.x() as f64)),
                0.5 * ((bv.bbox.min.y() as f64) + (bv.bbox.max.y() as f64)),
            );
            // BuildVolume.cpp:32
            // C++: m_circle.radius = 0.5 * m_bbox.size().cast<double>().norm();
            let s = bv.bbox.size();
            bv.circle.radius =
                0.5 * ((s.x() as f64) * (s.x() as f64) + (s.y() as f64) * (s.y() as f64)).sqrt();
        } else if printable_area.len() > 3 {
            // Circle was discretized, formatted into text with limited accuracy, thus the circle was deformed.
            // RANSAC is slightly more accurate than the iterative Taubin / Newton method with such an input.
            // BuildVolume.cpp:36
            // C++: //        m_circle = Geometry::circle_taubin_newton(printable_area);
            // BuildVolume.cpp:37
            // C++: m_circle = Geometry::circle_ransac(printable_area);
            bv.circle = crate::geometry::circle_ransac(&printable_area, 20, None);
            // BuildVolume.cpp:38
            // C++: bool is_circle = true;
            let mut is_circle = true;
            // BuildVolume.cpp:39-42 — NDEBUG only max_error measurement, omitted.
            // BuildVolume.cpp:43
            // C++: Vec2d prev = printable_area.back();
            let mut prev = *printable_area.last().unwrap();
            // BuildVolume.cpp:44
            // C++: for (const Vec2d &p : printable_area) {
            for p in &printable_area {
                // BuildVolume.cpp:45-47 — NDEBUG only, omitted.
                // BuildVolume.cpp:48-51
                // C++: if (std::abs((p - m_circle.center).norm() - m_circle.radius) > 0.005 ||
                // C++:     m_circle.radius - (0.5 * (prev + p) - m_circle.center).norm() > 3.) {
                if ((*p - bv.circle.center).length() - bv.circle.radius).abs() > 0.005
                    || bv.circle.radius - ((prev + *p) * 0.5 - bv.circle.center).length() > 3.0
                {
                    // BuildVolume.cpp:52
                    // C++: is_circle = false;
                    is_circle = false;
                    // BuildVolume.cpp:53
                    // C++: break;
                    break;
                }
                // BuildVolume.cpp:55
                // C++: prev = p;
                prev = *p;
            }
            // BuildVolume.cpp:57
            // C++: if (is_circle) {
            if is_circle {
                // BuildVolume.cpp:58
                // C++: m_type = Type::Circle;
                bv.volume_type = Type::Circle;
                // BuildVolume.cpp:59
                // C++: m_circle.center = scaled<double>(m_circle.center);
                bv.circle.center = Vec2d::new(
                    scaled_f64(bv.circle.center.x),
                    scaled_f64(bv.circle.center.y),
                );
                // BuildVolume.cpp:60
                // C++: m_circle.radius = scaled<double>(m_circle.radius);
                bv.circle.radius = scaled_f64(bv.circle.radius);
            }
        }

        // BuildVolume.cpp:64
        // C++: if (printable_area.size() >= 3 && m_type == Type::Invalid) {
        if printable_area.len() >= 3 && bv.volume_type == Type::Invalid {
            // Circle check is not used for Convex / Custom shapes, fill it with something reasonable.
            // BuildVolume.cpp:66
            // C++: m_circle = Geometry::smallest_enclosing_circle_welzl(m_convex_hull.points);
            bv.circle = crate::geometry::smallest_enclosing_circle_welzl(&bv.convex_hull.points);
            // BuildVolume.cpp:67
            // C++: m_type = (m_convex_hull.area() - m_area) < sqr(SCALED_EPSILON) ? Type::Convex : Type::Custom;
            bv.volume_type = if (bv.convex_hull.area() - bv.area) < sqr(SCALED_EPSILON) {
                Type::Convex
            } else {
                Type::Custom
            };
            // Initialize the top / bottom decomposition for inside convex polygon check. Do it with two different epsilons applied.
            // BuildVolume.cpp:69-76 — convex_decomposition lambda.
            // BuildVolume.cpp:77
            // C++: m_top_bottom_convex_hull_decomposition_scene = convex_decomposition(m_convex_hull, SceneEpsilon);
            bv.top_bottom_convex_hull_decomposition_scene =
                convex_decomposition(&bv.convex_hull, SCENE_EPSILON);
            // BuildVolume.cpp:78
            // C++: m_top_bottom_convex_hull_decomposition_bed   = convex_decomposition(m_convex_hull, BedEpsilon);
            bv.top_bottom_convex_hull_decomposition_bed =
                convex_decomposition(&bv.convex_hull, BED_EPSILON);
        }

        // BuildVolume.cpp:81
        // C++: if (m_extruder_shapes.size() > 0)
        if !bv.extruder_shapes.is_empty() {
            // BuildVolume.cpp:83-86
            // C++: m_shared_volume.data[0] = m_bboxf.min.x();  ...
            bv.shared_volume.data[0] = bv.bboxf.min.x as f32;
            bv.shared_volume.data[1] = bv.bboxf.min.y as f32;
            bv.shared_volume.data[2] = bv.bboxf.max.x as f32;
            bv.shared_volume.data[3] = bv.bboxf.max.y as f32;
            // BuildVolume.cpp:87
            // C++: m_shared_volume.zs[1] = m_bboxf.max.z();
            bv.shared_volume.zs[1] = bv.bboxf.max.z as f32;
            // BuildVolume.cpp:88
            // C++: for (unsigned int index = 0; index < m_extruder_shapes.size(); index++)
            let num_extruders = bv.extruder_shapes.len();
            for index in 0..num_extruders {
                // BuildVolume.cpp:90
                // C++: std::vector<Vec2d>& extruder_shape = m_extruder_shapes[index];
                let extruder_shape = bv.extruder_shapes[index].clone();
                // BuildVolume.cpp:91
                // C++: BuildExtruderVolume extruder_volume;
                let mut extruder_volume = BuildExtruderVolume::new();

                // BuildVolume.cpp:93
                // C++: if (extruder_shape.empty())
                if extruder_shape.is_empty() {
                    // should not happen
                    // BuildVolume.cpp:96
                    // C++: BOOST_LOG_TRIVIAL(warning) << boost::format("Found invalid extruder_printable_area of index %1%")%index;
                    log::warn!("Found invalid extruder_printable_area of index {}", index);
                    // BuildVolume.cpp:97
                    // C++: assert(false);
                    // BuildVolume.cpp:98
                    // C++: m_extruder_shapes.clear();
                    bv.extruder_shapes.clear();
                    // BuildVolume.cpp:99
                    // C++: return;
                    return bv;
                }

                // BuildVolume.cpp:102
                // C++: if ((extruder_shape == printable_area)&&(extruder_printable_heights[index] == printable_height)) {
                if extruder_shape == printable_area
                    && extruder_printable_heights[index] == printable_height
                {
                    // BuildVolume.cpp:103
                    // C++: extruder_volume.same_with_bed = true;
                    extruder_volume.same_with_bed = true;
                    // BuildVolume.cpp:104
                    // C++: extruder_volume.type = m_type;
                    extruder_volume.volume_type = bv.volume_type;
                    // BuildVolume.cpp:105
                    // C++: extruder_volume.bbox = m_bbox;
                    extruder_volume.bbox = bv.bbox;
                    // BuildVolume.cpp:106
                    // C++: extruder_volume.bboxf = m_bboxf;
                    extruder_volume.bboxf = bv.bboxf;
                    // BuildVolume.cpp:107
                    // C++: extruder_volume.circle = m_circle;
                    extruder_volume.circle = bv.circle;
                } else {
                    // BuildVolume.cpp:110
                    // C++: Polygon poly = Polygon::new_scale(extruder_shape);
                    let poly = Polygon::from_points(
                        extruder_shape
                            .iter()
                            .map(|p| Point::new(scale(p.x), scale(p.y)))
                            .collect(),
                    );

                    // BuildVolume.cpp:112
                    // C++: double poly_area = poly.area();
                    let poly_area = poly.area();
                    // BuildVolume.cpp:113
                    // C++: extruder_volume.bbox = get_extents(poly);
                    extruder_volume.bbox = poly.bounding_box();
                    // BuildVolume.cpp:114
                    // C++: BoundingBoxf temp_bboxf = get_extents(extruder_shape);
                    let temp_bboxf: BoundingBoxf =
                        BoundingBoxf::new_from_points_slice(&extruder_shape);
                    // BuildVolume.cpp:115
                    // C++: extruder_volume.bboxf = BoundingBoxf3{ to_3d(temp_bboxf.min, 0.), to_3d(temp_bboxf.max, extruder_printable_heights[index]) };
                    extruder_volume.bboxf = BoundingBoxf3::new_from_points(
                        Vec3d::new(temp_bboxf.min.x, temp_bboxf.min.y, 0.0),
                        Vec3d::new(
                            temp_bboxf.max.x,
                            temp_bboxf.max.y,
                            extruder_printable_heights[index],
                        ),
                    );

                    // BuildVolume.cpp:117
                    // C++: if (extruder_shape.size() >= 4 && std::abs((poly_area - double(extruder_volume.bbox.size().x()) * double(extruder_volume.bbox.size().y()))) < sqr(SCALED_EPSILON))
                    if extruder_shape.len() >= 4
                        && (poly_area
                            - (extruder_volume.bbox.size().x() as f64)
                                * (extruder_volume.bbox.size().y() as f64))
                            .abs()
                            < sqr(SCALED_EPSILON)
                    {
                        // BuildVolume.cpp:119
                        // C++: extruder_volume.type = Type::Rectangle;
                        extruder_volume.volume_type = Type::Rectangle;
                        // BuildVolume.cpp:120
                        // C++: extruder_volume.circle.center = 0.5 * (extruder_volume.bbox.min.cast<double>() + extruder_volume.bbox.max.cast<double>());
                        extruder_volume.circle.center = Vec2d::new(
                            0.5 * ((extruder_volume.bbox.min.x() as f64)
                                + (extruder_volume.bbox.max.x() as f64)),
                            0.5 * ((extruder_volume.bbox.min.y() as f64)
                                + (extruder_volume.bbox.max.y() as f64)),
                        );
                        // BuildVolume.cpp:121
                        // C++: extruder_volume.circle.radius = 0.5 * extruder_volume.bbox.size().cast<double>().norm();
                        let es = extruder_volume.bbox.size();
                        extruder_volume.circle.radius = 0.5
                            * ((es.x() as f64) * (es.x() as f64)
                                + (es.y() as f64) * (es.y() as f64))
                                .sqrt();
                    } else if extruder_shape.len() > 3 {
                        // BuildVolume.cpp:124
                        // C++: extruder_volume.circle = Geometry::circle_ransac(extruder_shape);
                        extruder_volume.circle = crate::geometry::circle_ransac(&extruder_shape, 20, None);
                        // BuildVolume.cpp:125
                        // C++: bool is_circle = true;
                        let mut is_circle = true;
                        // BuildVolume.cpp:127
                        // C++: Vec2d prev = extruder_shape.back();
                        let mut prev = *extruder_shape.last().unwrap();
                        // BuildVolume.cpp:128
                        // C++: for (const Vec2d &p : extruder_shape) {
                        for p in &extruder_shape {
                            // BuildVolume.cpp:129-132
                            // C++: if (std::abs((p - extruder_volume.circle.center).norm() - extruder_volume.circle.radius) > 0.005 ||
                            // C++:     extruder_volume.circle.radius - (0.5 * (prev + p) -extruder_volume.circle.center).norm() > 3.) {
                            if ((*p - extruder_volume.circle.center).length()
                                - extruder_volume.circle.radius)
                                .abs()
                                > 0.005
                                || extruder_volume.circle.radius
                                    - ((prev + *p) * 0.5 - extruder_volume.circle.center).length()
                                    > 3.0
                            {
                                // BuildVolume.cpp:133
                                // C++: is_circle = false;
                                is_circle = false;
                                // BuildVolume.cpp:134
                                // C++: break;
                                break;
                            }
                            // BuildVolume.cpp:136
                            // C++: prev = p;
                            prev = *p;
                        }
                        // BuildVolume.cpp:138
                        // C++: if (is_circle) {
                        if is_circle {
                            // BuildVolume.cpp:139
                            // C++: extruder_volume.type = Type::Circle;
                            extruder_volume.volume_type = Type::Circle;
                            // BuildVolume.cpp:140
                            // C++: extruder_volume.circle.center = scaled<double>(extruder_volume.circle.center);
                            extruder_volume.circle.center = Vec2d::new(
                                scaled_f64(extruder_volume.circle.center.x),
                                scaled_f64(extruder_volume.circle.center.y),
                            );
                            // BuildVolume.cpp:141
                            // C++: extruder_volume.circle.radius = scaled<double>(extruder_volume.circle.radius);
                            extruder_volume.circle.radius =
                                scaled_f64(extruder_volume.circle.radius);
                        }
                    }

                    // BuildVolume.cpp:145
                    // C++: if (m_type == Type::Invalid) {
                    if bv.volume_type == Type::Invalid {
                        // not supported currently, use the same as bed
                        // BuildVolume.cpp:147
                        // C++: extruder_volume.same_with_bed = true;
                        extruder_volume.same_with_bed = true;
                        // BuildVolume.cpp:148
                        // C++: extruder_volume.type = m_type;
                        extruder_volume.volume_type = bv.volume_type;
                        // BuildVolume.cpp:149
                        // C++: extruder_volume.bbox = m_bbox;
                        extruder_volume.bbox = bv.bbox;
                        // BuildVolume.cpp:150
                        // C++: extruder_volume.bboxf = m_bboxf;
                        extruder_volume.bboxf = bv.bboxf;
                        // BuildVolume.cpp:151
                        // C++: extruder_volume.circle = m_circle;
                        extruder_volume.circle = bv.circle;
                    }
                    // always ignore z
                    // BuildVolume.cpp:154
                    // C++: extruder_volume.bboxf.min.z() = -std::numeric_limits<double>::max();
                    extruder_volume.bboxf.min.z = -f64::MAX;
                }
                // BuildVolume.cpp:156
                // C++: m_extruder_volumes.push_back(std::move(extruder_volume));
                bv.extruder_volumes.push(extruder_volume.clone());

                // BuildVolume.cpp:158-159
                // C++: if (m_shared_volume.data[0] < extruder_volume.bboxf.min.x())
                // C++:     m_shared_volume.data[0] = extruder_volume.bboxf.min.x();
                if bv.shared_volume.data[0] < extruder_volume.bboxf.min.x as f32 {
                    bv.shared_volume.data[0] = extruder_volume.bboxf.min.x as f32;
                }
                // BuildVolume.cpp:160-161
                if bv.shared_volume.data[1] < extruder_volume.bboxf.min.y as f32 {
                    bv.shared_volume.data[1] = extruder_volume.bboxf.min.y as f32;
                }
                // BuildVolume.cpp:162-163
                if bv.shared_volume.data[2] > extruder_volume.bboxf.max.x as f32 {
                    bv.shared_volume.data[2] = extruder_volume.bboxf.max.x as f32;
                }
                // BuildVolume.cpp:164-165
                if bv.shared_volume.data[3] > extruder_volume.bboxf.max.y as f32 {
                    bv.shared_volume.data[3] = extruder_volume.bboxf.max.y as f32;
                }
                // BuildVolume.cpp:166-167
                if bv.shared_volume.zs[1] > extruder_volume.bboxf.max.z as f32 {
                    bv.shared_volume.zs[1] = extruder_volume.bboxf.max.z as f32;
                }
            }

            // BuildVolume.cpp:170
            // C++: m_shared_volume.type = static_cast<int>(m_type);
            bv.shared_volume.volume_type = bv.volume_type.to_int();
            // BuildVolume.cpp:171
            // C++: m_shared_volume.zs[0] = 0.f;
            bv.shared_volume.zs[0] = 0.0;
            // BuildVolume.cpp:172
            // C++: //m_shared_volume.zs[1] = printable_height;
        }

        // BuildVolume.cpp:175
        // C++: BOOST_LOG_TRIVIAL(debug) << "BuildVolume printable_area clasified as: " << this->type_name();
        log::debug!("BuildVolume printable_area clasified as: {}", bv.type_name());

        bv
    }

    /// Source data, unscaled coordinates.
    /// BuildVolume.hpp:62
    /// C++: const std::vector<Vec2d>& printable_area() const { return m_bed_shape; }
    pub fn printable_area(&self) -> &[Vec2d] {
        &self.bed_shape
    }

    /// BuildVolume.hpp:63
    /// C++: double printable_height() const { return m_max_print_height; }
    pub fn printable_height(&self) -> f64 {
        self.max_print_height
    }

    /// BuildVolume.hpp:64
    /// C++: const std::vector<std::vector<Vec2d>>& extruder_areas() const { return m_extruder_shapes; }
    pub fn extruder_areas(&self) -> &[Vec<Vec2d>] {
        &self.extruder_shapes
    }

    /// BuildVolume.hpp:65
    /// C++: const std::vector<double>& extruder_heights() const { return m_extruder_printable_height; }
    pub fn extruder_heights(&self) -> &[f64] {
        &self.extruder_printable_height
    }

    /// BuildVolume.hpp:66
    /// C++: const BuildSharedVolume& get_shared_volume() const { return m_shared_volume; }
    pub fn get_shared_volume(&self) -> &BuildSharedVolume {
        &self.shared_volume
    }

    /// Derived data
    /// BuildVolume.hpp:69
    /// C++: Type type() const { return m_type; }
    pub fn volume_type(&self) -> Type {
        self.volume_type
    }

    /// Format the type for console output (static).
    /// BuildVolume.cpp:594-607
    /// C++: std::string_view BuildVolume::type_name(Type type)
    pub fn type_name_of(type_: Type) -> &'static str {
        // BuildVolume.cpp:597
        match type_ {
            // BuildVolume.cpp:598
            // C++: case Type::Invalid:   return "Invalid"sv;
            Type::Invalid => "Invalid",
            // BuildVolume.cpp:599
            // C++: case Type::Rectangle: return "Rectangle"sv;
            Type::Rectangle => "Rectangle",
            // BuildVolume.cpp:600
            // C++: case Type::Circle:    return "Circle"sv;
            Type::Circle => "Circle",
            // BuildVolume.cpp:601
            // C++: case Type::Convex:    return "Convex"sv;
            Type::Convex => "Convex",
            // BuildVolume.cpp:602
            // C++: case Type::Custom:    return "Custom"sv;
            Type::Custom => "Custom",
        }
    }

    /// BuildVolume.hpp:72
    /// C++: std::string_view type_name() const { return type_name(m_type); }
    pub fn type_name(&self) -> &'static str {
        Self::type_name_of(self.volume_type)
    }

    /// BuildVolume.hpp:73
    /// C++: bool valid() const { return m_type != Type::Invalid; }
    pub fn valid(&self) -> bool {
        self.volume_type != Type::Invalid
    }

    /// BuildVolume.hpp:75
    /// C++: const Polygon& polygon() const { return m_polygon; }
    pub fn polygon(&self) -> &Polygon {
        &self.polygon
    }

    /// BuildVolume.hpp:77
    /// C++: const BoundingBox& bounding_box() const { return m_bbox; }
    pub fn bounding_box(&self) -> &BoundingBox {
        &self.bbox
    }

    /// BuildVolume.hpp:79
    /// C++: const BoundingBoxf3& bounding_volume() const { return m_bboxf; }
    pub fn bounding_volume(&self) -> &BoundingBoxf3 {
        &self.bboxf
    }

    /// BuildVolume.hpp:80
    /// C++: BoundingBoxf bounding_volume2d() const { return { to_2d(m_bboxf.min), to_2d(m_bboxf.max) }; }
    pub fn bounding_volume2d(&self) -> BoundingBoxf {
        BoundingBoxf::new_from_points(
            Vec2d::new(self.bboxf.min.x, self.bboxf.min.y),
            Vec2d::new(self.bboxf.max.x, self.bboxf.max.y),
        )
    }

    /// Center of the print bed, unscaled.
    /// BuildVolume.hpp:84
    /// C++: Vec2d bed_center() const { return to_2d(m_bboxf.center()); }
    pub fn bed_center(&self) -> Vec2d {
        let center = self.bboxf.center();
        Vec2d::new(center.x, center.y)
    }

    /// BuildVolume.hpp:86
    /// C++: const Polygon& convex_hull() const { return m_convex_hull; }
    pub fn convex_hull(&self) -> &Polygon {
        &self.convex_hull
    }

    /// BuildVolume.hpp:88
    /// C++: const Geometry::Circled& circle() const { return m_circle; }
    pub fn circle(&self) -> &Circle {
        &self.circle
    }

    /// Called by Plater to update Inside / Colliding / Outside state of ModelObjects before slicing.
    /// Using SceneEpsilon.
    /// BuildVolume.cpp:369-402
    /// C++: BuildVolume::ObjectState BuildVolume::object_state(const indexed_triangle_set& its, const Transform3f& trafo, bool may_be_below_bed, bool ignore_bottom) const
    pub fn object_state(
        &self,
        its: &indexed_triangle_set,
        trafo: &Transform3D,
        may_be_below_bed: bool,
        ignore_bottom: bool,
    ) -> ObjectState {
        // BuildVolume.cpp:371
        // C++: switch (m_type) {
        match self.volume_type {
            // BuildVolume.cpp:372
            Type::Rectangle => {
                // BuildVolume.cpp:374
                // C++: BoundingBox3Base<Vec3d> build_volume = this->bounding_volume().inflated(SceneEpsilon);
                let mut build_volume = self.bounding_volume().inflated(SCENE_EPSILON);
                // BuildVolume.cpp:375-376
                // C++: if (m_max_print_height == 0.0) build_volume.max.z() = std::numeric_limits<double>::max();
                if self.max_print_height == 0.0 {
                    build_volume.max.z = f64::MAX;
                }
                // BuildVolume.cpp:377-378
                // C++: if (ignore_bottom) build_volume.min.z() = -std::numeric_limits<double>::max();
                if ignore_bottom {
                    build_volume.min.z = -f64::MAX;
                }
                // BuildVolume.cpp:379
                // C++: BoundingBox3Base<Vec3f> build_volumef(build_volume.min.cast<float>(), build_volume.max.cast<float>());
                // (kept as f64 here; comparisons are equivalent)
                let build_volumef = build_volume;
                // BuildVolume.cpp:383
                // C++: return object_state_templ(its, trafo, may_be_below_bed, [build_volumef](const Vec3f &pt) { return build_volumef.contains(pt); });
                object_state_templ(its, trafo, may_be_below_bed, |pt| {
                    build_volumef.contains_point(Vec3d::new(pt.x, pt.y, pt.z))
                })
            }
            // BuildVolume.cpp:385
            Type::Circle => {
                // BuildVolume.cpp:387
                // C++: Geometry::Circlef circle { unscaled<float>(m_circle.center), unscaled<float>(m_circle.radius + SceneEpsilon) };
                let circle = Circle::new(
                    Vec2d::new(
                        unscaled_f64(self.circle.center.x),
                        unscaled_f64(self.circle.center.y),
                    ),
                    unscaled_f64(self.circle.radius) + SCENE_EPSILON,
                );
                // BuildVolume.cpp:388-390
                // C++: return m_max_print_height == 0.0 ?
                if self.max_print_height == 0.0 {
                    object_state_templ(its, trafo, may_be_below_bed, |pt| {
                        circle.contains(Vec2d::new(pt.x, pt.y))
                    })
                } else {
                    let z = self.max_print_height + SCENE_EPSILON;
                    object_state_templ(its, trafo, may_be_below_bed, |pt| {
                        (pt.z as f64) < z && circle.contains(Vec2d::new(pt.x, pt.y))
                    })
                }
            }
            // BuildVolume.cpp:392-394
            // C++: case Type::Convex: case Type::Custom:
            Type::Convex | Type::Custom => {
                // BuildVolume.cpp:395-397
                if self.max_print_height == 0.0 {
                    object_state_templ(its, trafo, may_be_below_bed, |pt| {
                        inside_convex_polygon(
                            &self.top_bottom_convex_hull_decomposition_scene,
                            &Vec2d::new(pt.x as f64, pt.y as f64),
                        )
                    })
                } else {
                    let z = self.max_print_height + SCENE_EPSILON;
                    object_state_templ(its, trafo, may_be_below_bed, |pt| {
                        (pt.z as f64) < z
                            && inside_convex_polygon(
                                &self.top_bottom_convex_hull_decomposition_scene,
                                &Vec2d::new(pt.x as f64, pt.y as f64),
                            )
                    })
                }
            }
            // BuildVolume.cpp:398-400
            // C++: case Type::Invalid: default: return ObjectState::Inside;
            Type::Invalid => ObjectState::Inside,
        }
    }

    /// Called by GLVolumeCollection::check_outside_state() for a rectangular bed.
    /// BuildVolume.cpp:404-415
    /// C++: BuildVolume::ObjectState BuildVolume::volume_state_bbox(const BoundingBoxf3& volume_bbox, bool ignore_bottom) const
    pub fn volume_state_bbox(&self, volume_bbox: &BoundingBoxf3, ignore_bottom: bool) -> ObjectState {
        // BuildVolume.cpp:406
        // C++: assert(m_type == Type::Rectangle);
        debug_assert!(self.volume_type == Type::Rectangle);
        // BuildVolume.cpp:407
        // C++: BoundingBox3Base<Vec3d> build_volume = this->bounding_volume().inflated(SceneEpsilon);
        let mut build_volume = self.bounding_volume().inflated(SCENE_EPSILON);
        // BuildVolume.cpp:408-409
        // C++: if (m_max_print_height == 0.0) build_volume.max.z() = std::numeric_limits<double>::max();
        if self.max_print_height == 0.0 {
            build_volume.max.z = f64::MAX;
        }
        // BuildVolume.cpp:410-411
        // C++: if (ignore_bottom) build_volume.min.z() = -std::numeric_limits<double>::max();
        if ignore_bottom {
            build_volume.min.z = -f64::MAX;
        }
        // BuildVolume.cpp:412-414
        // C++: return build_volume.max.z() <= - SceneEpsilon ? ObjectState::Below :
        // C++:        build_volume.contains(volume_bbox) ? ObjectState::Inside :
        // C++:        build_volume.intersects(volume_bbox) ? ObjectState::Colliding : ObjectState::Outside;
        if build_volume.max.z <= -SCENE_EPSILON {
            ObjectState::Below
        } else if build_volume.contains_bb(volume_bbox) {
            ObjectState::Inside
        } else if build_volume.intersects(volume_bbox) {
            ObjectState::Colliding
        } else {
            ObjectState::Outside
        }
    }

    /// BuildVolume.hpp:125
    /// C++: int get_extruder_area_count() const { return m_extruder_volumes.size(); }
    pub fn get_extruder_area_count(&self) -> i32 {
        self.extruder_volumes.len() as i32
    }

    /// BuildVolume.cpp:417-421
    /// C++: const BuildVolume::BuildExtruderVolume& BuildVolume::get_extruder_area_volume(int index) const
    pub fn get_extruder_area_volume(&self, index: i32) -> &BuildExtruderVolume {
        // BuildVolume.cpp:419
        // C++: assert(index >= 0 && index < m_extruder_volumes.size());
        assert!(index >= 0 && (index as usize) < self.extruder_volumes.len());
        // BuildVolume.cpp:420
        // C++: return m_extruder_volumes[index];
        &self.extruder_volumes[index as usize]
    }

    /// BuildVolume.cpp:423-458
    /// C++: BuildVolume::ObjectState BuildVolume::check_object_state_with_extruder_area(const indexed_triangle_set &its, const Transform3f &trafo, int index) const
    pub fn check_object_state_with_extruder_area(
        &self,
        its: &indexed_triangle_set,
        trafo: &Transform3D,
        index: i32,
    ) -> ObjectState {
        // BuildVolume.cpp:425
        // C++: const BuildExtruderVolume& extruder_volume = get_extruder_area_volume(index);
        let extruder_volume = self.get_extruder_area_volume(index);
        // BuildVolume.cpp:426
        // C++: ObjectState return_state = ObjectState::Inside;
        let mut return_state = ObjectState::Inside;

        // BuildVolume.cpp:428
        // C++: if (!extruder_volume.same_with_bed) {
        if !extruder_volume.same_with_bed {
            // BuildVolume.cpp:429
            // C++: switch (extruder_volume.type) {
            match extruder_volume.volume_type {
                // BuildVolume.cpp:430
                Type::Rectangle => {
                    // BuildVolume.cpp:432
                    // C++: BoundingBox3Base<Vec3d> build_volume = extruder_volume.bboxf.inflated(SceneEpsilon);
                    let mut build_volume = extruder_volume.bboxf.inflated(SCENE_EPSILON);
                    // BuildVolume.cpp:433-434
                    // C++: if (m_max_print_height == 0.0) build_volume.max.z() = std::numeric_limits<double>::max();
                    if self.max_print_height == 0.0 {
                        build_volume.max.z = f64::MAX;
                    }
                    // BuildVolume.cpp:435
                    // C++: BoundingBox3Base<Vec3f> build_volumef(build_volume.min.cast<float>(), build_volume.max.cast<float>());
                    let build_volumef = build_volume;
                    // BuildVolume.cpp:437
                    // C++: return_state = object_state_templ(its, trafo, false, [build_volumef](const Vec3f &pt) { return build_volumef.contains(pt); });
                    return_state = object_state_templ(its, trafo, false, |pt| {
                        build_volumef.contains_point(Vec3d::new(pt.x, pt.y, pt.z))
                    });
                }
                // BuildVolume.cpp:440
                Type::Circle => {
                    // BuildVolume.cpp:442
                    // C++: Geometry::Circlef circle { unscaled<float>(extruder_volume.circle.center), unscaled<float>(extruder_volume.circle.radius + SceneEpsilon) };
                    let circle = Circle::new(
                        Vec2d::new(
                            unscaled_f64(extruder_volume.circle.center.x),
                            unscaled_f64(extruder_volume.circle.center.y),
                        ),
                        unscaled_f64(extruder_volume.circle.radius) + SCENE_EPSILON,
                    );
                    // BuildVolume.cpp:443-445
                    // C++: return_state = (m_max_print_height == 0.0) ? ...
                    return_state = if self.max_print_height == 0.0 {
                        object_state_templ(its, trafo, false, |pt| {
                            circle.contains(Vec2d::new(pt.x, pt.y))
                        })
                    } else {
                        let z = self.max_print_height + SCENE_EPSILON;
                        object_state_templ(its, trafo, false, |pt| {
                            (pt.z as f64) < z && circle.contains(Vec2d::new(pt.x, pt.y))
                        })
                    };
                }
                // BuildVolume.cpp:448-450
                // C++: case Type::Invalid: default: break;
                _ => {}
            }
        }

        // BuildVolume.cpp:454
        // C++: if (return_state != ObjectState::Inside)
        if return_state != ObjectState::Inside {
            // BuildVolume.cpp:455
            // C++: return_state = ObjectState::Limited;
            return_state = ObjectState::Limited;
        }

        // BuildVolume.cpp:457
        // C++: return return_state;
        return_state
    }

    /// BuildVolume.cpp:460-476
    /// C++: BuildVolume::ObjectState BuildVolume::check_object_state_with_extruder_areas(const indexed_triangle_set &its, const Transform3f &trafo, std::vector<bool>& inside_extruders) const
    pub fn check_object_state_with_extruder_areas(
        &self,
        its: &indexed_triangle_set,
        trafo: &Transform3D,
        inside_extruders: &mut Vec<bool>,
    ) -> ObjectState {
        // BuildVolume.cpp:462
        // C++: ObjectState result = ObjectState::Inside;
        let mut result = ObjectState::Inside;
        // BuildVolume.cpp:463
        // C++: int extruder_area_count = get_extruder_area_count();
        let extruder_area_count = self.get_extruder_area_count();
        // BuildVolume.cpp:464
        // C++: inside_extruders.resize(extruder_area_count, true);
        inside_extruders.resize(extruder_area_count as usize, true);
        // BuildVolume.cpp:465
        // C++: for (int index = 0; index < extruder_area_count; index++)
        for index in 0..extruder_area_count {
            // BuildVolume.cpp:467
            // C++: ObjectState state = check_object_state_with_extruder_area(its, trafo, index);
            let state = self.check_object_state_with_extruder_area(its, trafo, index);

            // BuildVolume.cpp:469
            // C++: if (state == ObjectState::Limited) {
            if state == ObjectState::Limited {
                // BuildVolume.cpp:470
                // C++: inside_extruders[index] = false;
                inside_extruders[index as usize] = false;
                // BuildVolume.cpp:471
                // C++: result = ObjectState::Limited;
                result = ObjectState::Limited;
            }
        }

        // BuildVolume.cpp:475
        // C++: return result;
        result
    }

    /// BuildVolume.cpp:478-486
    /// C++: BuildVolume::ObjectState BuildVolume::check_volume_bbox_state_with_extruder_area(const BoundingBoxf3& volume_bbox, int index) const
    pub fn check_volume_bbox_state_with_extruder_area(
        &self,
        volume_bbox: &BoundingBoxf3,
        index: i32,
    ) -> ObjectState {
        // BuildVolume.cpp:480
        // C++: const BuildExtruderVolume& extruder_volume = get_extruder_area_volume(index);
        let extruder_volume = self.get_extruder_area_volume(index);
        // BuildVolume.cpp:481
        // C++: BoundingBox3Base<Vec3d> extruder_bbox = extruder_volume.bboxf.inflated(SceneEpsilon);
        let extruder_bbox = extruder_volume.bboxf.inflated(SCENE_EPSILON);
        // BuildVolume.cpp:482-485
        // C++: if (extruder_volume.same_with_bed || extruder_bbox.contains(volume_bbox)) return ObjectState::Inside; else return ObjectState::Limited;
        if extruder_volume.same_with_bed || extruder_bbox.contains_bb(volume_bbox) {
            ObjectState::Inside
        } else {
            ObjectState::Limited
        }
    }

    /// BuildVolume.cpp:488-504
    /// C++: BuildVolume::ObjectState BuildVolume::check_volume_bbox_state_with_extruder_areas(const BoundingBoxf3& volume_bbox, std::vector<bool>& inside_extruders) const
    pub fn check_volume_bbox_state_with_extruder_areas(
        &self,
        volume_bbox: &BoundingBoxf3,
        inside_extruders: &mut Vec<bool>,
    ) -> ObjectState {
        // BuildVolume.cpp:490
        // C++: ObjectState result = ObjectState::Inside;
        let mut result = ObjectState::Inside;
        // BuildVolume.cpp:491
        // C++: int extruder_area_count = get_extruder_area_count();
        let extruder_area_count = self.get_extruder_area_count();
        // BuildVolume.cpp:492
        // C++: inside_extruders.resize(extruder_area_count, true);
        inside_extruders.resize(extruder_area_count as usize, true);
        // BuildVolume.cpp:493
        // C++: for (int index = 0; index < extruder_area_count; index++)
        for index in 0..extruder_area_count {
            // BuildVolume.cpp:495
            // C++: ObjectState state = check_volume_bbox_state_with_extruder_area(volume_bbox, index);
            let state = self.check_volume_bbox_state_with_extruder_area(volume_bbox, index);

            // BuildVolume.cpp:497
            // C++: if (state == ObjectState::Limited) {
            if state == ObjectState::Limited {
                // BuildVolume.cpp:498
                // C++: inside_extruders[index] = false;
                inside_extruders[index as usize] = false;
                // BuildVolume.cpp:499
                // C++: result = ObjectState::Limited;
                result = ObjectState::Limited;
            }
        }

        // BuildVolume.cpp:503
        // C++: return result;
        result
    }

    /// Called on initial G-code preview on OpenGL vertex buffer interleaved normals and vertices.
    /// BuildVolume.cpp:560-592
    /// C++: bool BuildVolume::all_paths_inside_vertices_and_normals_interleaved(const std::vector<float>& paths, const Eigen::AlignedBox<float, 3>& paths_bbox, bool ignore_bottom) const
    pub fn all_paths_inside_vertices_and_normals_interleaved(
        &self,
        paths: &[f32],
        paths_bbox_min: Point3F,
        paths_bbox_max: Point3F,
        ignore_bottom: bool,
    ) -> bool {
        // BuildVolume.cpp:562
        // C++: assert(paths.size() % 6 == 0);
        debug_assert!(paths.len() % 6 == 0);
        // BuildVolume.cpp:563
        // C++: static constexpr const double epsilon = BedEpsilon;
        const EPSILON_LOCAL: f64 = BED_EPSILON;
        // BuildVolume.cpp:564
        // C++: switch (m_type) {
        match self.volume_type {
            // BuildVolume.cpp:565
            Type::Rectangle => {
                // BuildVolume.cpp:567
                // C++: BoundingBox3Base<Vec3d> build_volume = this->bounding_volume().inflated(epsilon);
                let mut build_volume = self.bounding_volume().inflated(EPSILON_LOCAL);
                // BuildVolume.cpp:568-569
                if self.max_print_height == 0.0 {
                    build_volume.max.z = f64::MAX;
                }
                // BuildVolume.cpp:570-571
                if ignore_bottom {
                    build_volume.min.z = -f64::MAX;
                }
                // BuildVolume.cpp:572
                // C++: return build_volume.contains(paths_bbox.min().cast<double>()) && build_volume.contains(paths_bbox.max().cast<double>());
                build_volume
                    .contains_point(Vec3d::new(paths_bbox_min.x, paths_bbox_min.y, paths_bbox_min.z))
                    && build_volume.contains_point(Vec3d::new(
                        paths_bbox_max.x,
                        paths_bbox_max.y,
                        paths_bbox_max.z,
                    ))
            }
            // BuildVolume.cpp:574
            Type::Circle => {
                // BuildVolume.cpp:576
                // C++: const Vec2f c = unscaled<float>(m_circle.center);
                let c = Vec2d::new(
                    unscaled_f64(self.circle.center.x),
                    unscaled_f64(self.circle.center.y),
                );
                // BuildVolume.cpp:577
                // C++: const float r = unscaled<double>(m_circle.radius) + float(epsilon);
                let r = unscaled_f64(self.circle.radius) + EPSILON_LOCAL;
                // BuildVolume.cpp:578
                // C++: const float r2 = sqr(r);
                let r2 = sqr(r);
                // BuildVolume.cpp:579-581
                if self.max_print_height == 0.0 {
                    all_inside_vertices_normals_interleaved(paths, |p| {
                        let d = Vec2d::new(p.x as f64 - c.x, p.y as f64 - c.y);
                        d.x * d.x + d.y * d.y <= r2
                    })
                } else {
                    let z = self.max_print_height + EPSILON_LOCAL;
                    all_inside_vertices_normals_interleaved(paths, |p| {
                        let d = Vec2d::new(p.x as f64 - c.x, p.y as f64 - c.y);
                        d.x * d.x + d.y * d.y <= r2 && (p.z as f64) <= z
                    })
                }
            }
            // BuildVolume.cpp:583-585
            // C++: case Type::Convex: case Type::Custom:
            Type::Convex | Type::Custom => {
                // BuildVolume.cpp:586-588
                if self.max_print_height == 0.0 {
                    all_inside_vertices_normals_interleaved(paths, |p| {
                        inside_convex_polygon(
                            &self.top_bottom_convex_hull_decomposition_bed,
                            &Vec2d::new(p.x as f64, p.y as f64),
                        )
                    })
                } else {
                    let z = self.max_print_height + EPSILON_LOCAL;
                    all_inside_vertices_normals_interleaved(paths, |p| {
                        inside_convex_polygon(
                            &self.top_bottom_convex_hull_decomposition_bed,
                            &Vec2d::new(p.x as f64, p.y as f64),
                        ) && (p.z as f64) <= z
                    })
                }
            }
            // BuildVolume.cpp:589-590
            // C++: default: return true;
            Type::Invalid => true,
        }
    }

    /// BuildVolume.cpp:609-618
    /// C++: indexed_triangle_set BuildVolume::bounding_mesh(bool scale) const
    pub fn bounding_mesh(&self, scale_flag: bool) -> indexed_triangle_set {
        // BuildVolume.cpp:611
        // C++: auto max_pt3 = m_bboxf.max;
        let max_pt3 = self.bboxf.max;
        // BuildVolume.cpp:612-617
        // C++: if (scale) {
        // C++:     return its_make_cube(scale_(max_pt3.x()), scale_(max_pt3.y()), scale_(max_pt3.z()));
        // C++: } else {
        // C++:     return its_make_cube(max_pt3.x(), max_pt3.y(), max_pt3.z());
        // C++: }
        // `its_make_cube` returns `normal_utils::indexed_triangle_set`; the field
        // types (`Vec<Vector3<f32>>` / `Vec<Vector3<i32>>`) are identical to this
        // module's `triangle_set_sampling::indexed_triangle_set`, so we move the
        // vectors across to satisfy the local return type.
        let cube = if scale_flag {
            // C++ `scale_(val) = val / SCALING_FACTOR` (SCALING_FACTOR = 1e-6) maps to
            // the crate's integer-scaling boundary `scale(val)` (SCALING_FACTOR = 1e5);
            // `its_make_cube` takes doubles, so re-cast the scaled coordinate to f64.
            its_make_cube(
                scale(max_pt3.x) as f64,
                scale(max_pt3.y) as f64,
                scale(max_pt3.z) as f64,
            )
        } else {
            its_make_cube(max_pt3.x, max_pt3.y, max_pt3.z)
        };
        indexed_triangle_set {
            vertices: cube.vertices,
            indices: cube.indices,
        }
    }

    // ----------------------------------------------------------------------
    // BLOCKED SYMBOLS (faithful port deferred — would require fakes today):
    //
    // BuildVolume.cpp:507-546
    //   bool BuildVolume::all_paths_inside(const GCodeProcessorResult& paths,
    //                                      const BoundingBoxf3& paths_bbox,
    //                                      bool ignore_bottom) const
    //   Blocked: depends on `GCodeProcessorResult::MoveVertex` (move.type,
    //   move.extrusion_role, move.width, move.height, move.position) and the
    //   `paths.moves` list. The crate's `gcode::g_code_processor::GCodeProcessorResult`
    //   is not yet fully ported — it exposes only the nested POD types
    //   (SliceWarning / FilamentUseInfo / SettingsIds / ...) and time/filament
    //   aggregates, not the per-move vertex list. Porting `move_valid()` / the
    //   per-move tests would require fabricating a `MoveVertex` type (a fake).
    //   Port once GCodeProcessor's move list is ported.
    // ----------------------------------------------------------------------
}

impl Default for BuildVolume {
    fn default() -> Self {
        Self::new()
    }
}

/// Trim the input transformed triangle mesh with print bed and test the remaining vertices with is_inside callback.
/// Return inside / colliding / outside state.
/// BuildVolume.cpp:284-367
/// C++: template<typename InsideFn>
/// C++: BuildVolume::ObjectState object_state_templ(const indexed_triangle_set &its, const Transform3f &trafo, bool may_be_below_bed, InsideFn is_inside)
fn object_state_templ<F>(
    its: &indexed_triangle_set,
    trafo: &Transform3D,
    may_be_below_bed: bool,
    is_inside: F,
) -> ObjectState
where
    F: Fn(Point3F) -> bool,
{
    // BuildVolume.cpp:289
    // C++: size_t num_inside = 0;
    let mut num_inside: usize = 0;
    // BuildVolume.cpp:290
    // C++: size_t num_above = 0;
    let mut num_above: usize = 0;
    // BuildVolume.cpp:291
    // C++: bool inside = false;
    let mut inside;
    // BuildVolume.cpp:292
    // C++: bool outside = false;
    let mut outside;
    // BuildVolume.cpp:293
    // C++: static constexpr const auto world_min_z = float(-BuildVolume::SceneEpsilon);
    let world_min_z = -SCENE_EPSILON;

    // Helper applying `trafo * v` to an f32 vertex, yielding an f64 point.
    let apply = |v: &Vec3f| -> Point3F {
        trafo.apply(Point3F::new(v.x as f64, v.y as f64, v.z as f64))
    };

    // BuildVolume.cpp:295
    // C++: if (may_be_below_bed)
    if may_be_below_bed {
        // Slower test, needs to clip the object edges with the print bed plane.
        // 1) Allocate transformed vertices with their position with respect to print bed surface.
        // BuildVolume.cpp:299
        // C++: std::vector<char> sides;
        let mut sides: Vec<i8> = Vec::with_capacity(its.vertices.len());

        // BuildVolume.cpp:302
        // C++: const auto sign = [](const stl_vertex& pt) { return pt.z() > world_min_z ? 1 : pt.z() < world_min_z ? -1 : 0; };
        let sign = |z: f64| -> i8 {
            if z > world_min_z {
                1
            } else if z < world_min_z {
                -1
            } else {
                0
            }
        };

        // BuildVolume.cpp:304
        // C++: for (const stl_vertex &v : its.vertices) {
        for v in &its.vertices {
            // BuildVolume.cpp:305
            // C++: const stl_vertex pt = trafo * v;
            let pt = apply(v);
            // BuildVolume.cpp:306
            // C++: const int s = sign(pt);
            let s = sign(pt.z);
            // BuildVolume.cpp:307
            // C++: sides.emplace_back(s);
            sides.push(s);
            // BuildVolume.cpp:308
            // C++: if (s >= 0) {
            if s >= 0 {
                // Vertex above or on print bed surface. Test whether it is inside the build volume.
                // BuildVolume.cpp:310
                // C++: ++ num_above;
                num_above += 1;
                // BuildVolume.cpp:311-312
                // C++: if (is_inside(pt)) ++ num_inside;
                if is_inside(pt) {
                    num_inside += 1;
                }
            }
        }

        // BuildVolume.cpp:316
        // C++: if (num_above == 0)
        if num_above == 0 {
            // Special case, the object is completely below the print bed, thus it is outside,
            // however we want to allow an object to be still printable if some of its parts are completely below the print bed.
            // BuildVolume.cpp:319
            // C++: return BuildVolume::ObjectState::Below;
            return ObjectState::Below;
        }

        // 2) Calculate intersections of triangle edges with the build surface.
        // BuildVolume.cpp:322
        // C++: inside = num_inside > 0;
        inside = num_inside > 0;
        // BuildVolume.cpp:323
        // C++: outside = num_inside < num_above;
        outside = num_inside < num_above;
        // BuildVolume.cpp:324
        // C++: if (num_above < its.vertices.size() && ! (inside && outside)) {
        if num_above < its.vertices.len() && !(inside && outside) {
            // Not completely above the build surface and status may still change by testing edges intersecting the build platform.
            // BuildVolume.cpp:326
            // C++: for (const stl_triangle_vertex_indices &tri : its.indices) {
            for tri in &its.indices {
                // BuildVolume.cpp:327
                // C++: const int s[3] = { sides[tri(0)], sides[tri(1)], sides[tri(2)] };
                let s = [
                    sides[tri.x as usize],
                    sides[tri.y as usize],
                    sides[tri.z as usize],
                ];
                // BuildVolume.cpp:328
                // C++: if (std::min(s[0], std::min(s[1], s[2])) < 0 && std::max(s[0], std::max(s[1], s[2])) > 0) {
                if s[0].min(s[1].min(s[2])) < 0 && s[0].max(s[1].max(s[2])) > 0 {
                    // Some edge of this triangle intersects the build platform. Calculate the intersection.
                    // BuildVolume.cpp:330
                    // C++: int iprev = 2;
                    let mut iprev = 2usize;
                    // BuildVolume.cpp:331
                    // C++: for (int iedge = 0; iedge < 3; ++ iedge) {
                    for iedge in 0..3usize {
                        // BuildVolume.cpp:332
                        // C++: if (s[iprev] * s[iedge] == -1) {
                        if (s[iprev] as i32) * (s[iedge] as i32) == -1 {
                            // edge intersects the build surface. Calculate intersection point.
                            // BuildVolume.cpp:334
                            // C++: const stl_vertex p1 = trafo * its.vertices[tri(iprev)];
                            let p1 = apply(&its.vertices[tri_index(tri, iprev) as usize]);
                            // BuildVolume.cpp:335
                            // C++: const stl_vertex p2 = trafo * its.vertices[tri(iedge)];
                            let p2 = apply(&its.vertices[tri_index(tri, iedge) as usize]);
                            // BuildVolume.cpp:336-338 — assertions, omitted in release.
                            // Edge crosses the z plane. Calculate intersection point with the plane.
                            // BuildVolume.cpp:340
                            // C++: const float t = (world_min_z - p1.z()) / (p2.z() - p1.z());
                            let t = (world_min_z - p1.z) / (p2.z - p1.z);
                            // BuildVolume.cpp:341
                            // C++: (is_inside(Vec3f(p1.x() + (p2.x() - p1.x()) * t, p1.y() + (p2.y() - p1.y()) * t, world_min_z)) ? inside : outside) = true;
                            let test_pt = Point3F::new(
                                p1.x + (p2.x - p1.x) * t,
                                p1.y + (p2.y - p1.y) * t,
                                world_min_z,
                            );
                            if is_inside(test_pt) {
                                inside = true;
                            } else {
                                outside = true;
                            }
                        }
                        // BuildVolume.cpp:343
                        // C++: iprev = iedge;
                        iprev = iedge;
                    }
                    // BuildVolume.cpp:345
                    // C++: if (inside && outside) break;
                    if inside && outside {
                        break;
                    }
                }
            }
        }
    } else {
        // Much simpler and faster code, not clipping the object with the print bed.
        // BuildVolume.cpp:354
        // C++: assert(! may_be_below_bed);
        debug_assert!(!may_be_below_bed);
        // BuildVolume.cpp:355
        // C++: num_above = its.vertices.size();
        num_above = its.vertices.len();
        // BuildVolume.cpp:356
        // C++: for (const stl_vertex &v : its.vertices) {
        for v in &its.vertices {
            // BuildVolume.cpp:357
            // C++: const stl_vertex pt = trafo * v;
            let pt = apply(v);
            // BuildVolume.cpp:358 — assert(pt.z() >= world_min_z), omitted in release.
            // BuildVolume.cpp:359-360
            // C++: if (is_inside(pt)) ++ num_inside;
            if is_inside(pt) {
                num_inside += 1;
            }
        }
        // BuildVolume.cpp:362
        // C++: inside = num_inside > 0;
        inside = num_inside > 0;
        // BuildVolume.cpp:363
        // C++: outside = num_inside < num_above;
        outside = num_inside < num_above;
    }

    // BuildVolume.cpp:366
    // C++: return inside ? (outside ? BuildVolume::ObjectState::Colliding : BuildVolume::ObjectState::Inside) : BuildVolume::ObjectState::Outside;
    if inside {
        if outside {
            ObjectState::Colliding
        } else {
            ObjectState::Inside
        }
    } else {
        ObjectState::Outside
    }
}

/// `tri(i)` index accessor matching C++ `stl_triangle_vertex_indices::operator()`.
#[inline]
fn tri_index(tri: &Vec3i, i: usize) -> i32 {
    match i {
        0 => tri.x,
        1 => tri.y,
        _ => tri.z,
    }
}

/// BuildVolume.cpp:548-558
/// C++: template<typename Fn>
/// C++: inline bool all_inside_vertices_normals_interleaved(const std::vector<float> &paths, Fn fn)
fn all_inside_vertices_normals_interleaved<F>(paths: &[f32], fn_: F) -> bool
where
    F: Fn(Point3F) -> bool,
{
    // BuildVolume.cpp:551
    // C++: for (auto it = paths.begin(); it != paths.end(); ) {
    let mut it = 0usize;
    while it != paths.len() {
        // BuildVolume.cpp:552
        // C++: it += 3;
        it += 3;
        // BuildVolume.cpp:553-554
        // C++: if (! fn({ *it, *(it + 1), *(it + 2) })) return false;
        if !fn_(Point3F::new(
            paths[it] as f64,
            paths[it + 1] as f64,
            paths[it + 2] as f64,
        )) {
            return false;
        }
        // BuildVolume.cpp:555
        // C++: it += 3;
        it += 3;
    }
    // BuildVolume.cpp:557
    // C++: return true;
    true
}

/// convex_decomposition lambda from BuildVolume.cpp:69-76.
/// C++: auto convex_decomposition = [](const Polygon &in, double epsilon) {
/// C++:     Polygon src = expand(in, float(epsilon)).front();
/// C++:     std::vector<Vec2d> pts;
/// C++:     pts.reserve(src.size());
/// C++:     for (const Point &pt : src.points)
/// C++:         pts.emplace_back(unscaled<double>(pt.cast<double>().eval()));
/// C++:     return Geometry::decompose_convex_polygon_top_bottom(pts);
/// C++: };
fn convex_decomposition(in_: &Polygon, epsilon: f64) -> (Vec<Vec2d>, Vec<Vec2d>) {
    // BuildVolume.cpp:70
    // C++: Polygon src = expand(in, float(epsilon)).front();
    let expanded = expand(in_, epsilon as f32);
    let src = if expanded.is_empty() {
        // expand() may legitimately return no contour for degenerate input;
        // `.front()` would be UB in C++, here decompose yields empty chains.
        return (Vec::new(), Vec::new());
    } else {
        &expanded[0]
    };
    // BuildVolume.cpp:71-72
    // C++: std::vector<Vec2d> pts; pts.reserve(src.size());
    let mut pts: Vec<Vec2d> = Vec::with_capacity(src.points.len());
    // BuildVolume.cpp:73-74
    // C++: for (const Point &pt : src.points)
    // C++:     pts.emplace_back(unscaled<double>(pt.cast<double>().eval()));
    for pt in &src.points {
        pts.push(Vec2d::new(unscale(pt.x()), unscale(pt.y())));
    }
    // BuildVolume.cpp:75
    // C++: return Geometry::decompose_convex_polygon_top_bottom(pts);
    decompose_convex_polygon_top_bottom(&pts)
}

/// `expand(const Polygon&, float)` from ClipperUtils — outward polygon offset.
/// Returns a vector of polygon contours (one per resulting island).
fn expand(polygon: &Polygon, delta: f32) -> Vec<Polygon> {
    // Mirror ClipperUtils `expand`: offset by `delta` (unscaled-to-scaled handled
    // by offset_polygons which expects an unscaled delta).
    let ex = clipper_utils::offset_polygons(
        std::slice::from_ref(polygon),
        delta as f64 / SCALING_FACTOR,
        OffsetJoinType::Miter,
    );
    to_polygons(&ex)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_build_volume_new() {
        // Test default constructor creates invalid volume
        // BuildVolume.hpp:57
        let bv = BuildVolume::new();
        assert_eq!(bv.volume_type(), Type::Invalid);
        assert!(!bv.valid());
    }

    #[test]
    fn test_build_volume_rectangle() {
        // Test rectangular bed detection
        // BuildVolume.cpp:28-32
        let area = vec![
            Vec2d::new(0.0, 0.0),
            Vec2d::new(200.0, 0.0),
            Vec2d::new(200.0, 200.0),
            Vec2d::new(0.0, 200.0),
        ];
        let bv = BuildVolume::new_from_config(area, 200.0, Vec::new(), Vec::new());
        assert_eq!(bv.volume_type(), Type::Rectangle);
        assert!(bv.valid());
        assert_eq!(bv.printable_height(), 200.0);
    }

    #[test]
    fn test_build_volume_circle() {
        // Test circular bed detection (72-gon, matching BedShapePanel::update_shape()).
        // BuildVolume.cpp:33-61
        let mut area = Vec::new();
        let center = Vec2d::new(100.0, 100.0);
        let radius = 100.0;
        for i in 0..72 {
            let angle = (i as f64) * 2.0 * std::f64::consts::PI / 72.0;
            area.push(Vec2d::new(
                center.x + radius * angle.cos(),
                center.y + radius * angle.sin(),
            ));
        }
        let bv = BuildVolume::new_from_config(area, 200.0, Vec::new(), Vec::new());
        assert_eq!(bv.volume_type(), Type::Circle);
        assert!(bv.valid());
    }

    #[test]
    fn test_object_state_values() {
        // Test ObjectState enum
        // BuildVolume.hpp:90-103
        assert_ne!(ObjectState::Inside, ObjectState::Colliding);
        assert_ne!(ObjectState::Colliding, ObjectState::Outside);
        assert_ne!(ObjectState::Outside, ObjectState::Below);
        assert_ne!(ObjectState::Below, ObjectState::Limited);
    }

    #[test]
    fn test_type_name() {
        // Test type name formatting
        // BuildVolume.cpp:594-607
        let bv = BuildVolume::new();
        assert_eq!(bv.type_name(), "Invalid");

        let area = vec![
            Vec2d::new(0.0, 0.0),
            Vec2d::new(200.0, 0.0),
            Vec2d::new(200.0, 200.0),
            Vec2d::new(0.0, 200.0),
        ];
        let bv = BuildVolume::new_from_config(area, 200.0, Vec::new(), Vec::new());
        assert_eq!(bv.type_name(), "Rectangle");
        assert_eq!(BuildVolume::type_name_of(Type::Circle), "Circle");
    }

    #[test]
    fn test_extruder_volumes() {
        // Test extruder area handling
        // BuildVolume.cpp:81-173
        let bed = vec![
            Vec2d::new(0.0, 0.0),
            Vec2d::new(200.0, 0.0),
            Vec2d::new(200.0, 200.0),
            Vec2d::new(0.0, 200.0),
        ];
        let extruder1 = vec![
            Vec2d::new(0.0, 0.0),
            Vec2d::new(100.0, 0.0),
            Vec2d::new(100.0, 200.0),
            Vec2d::new(0.0, 200.0),
        ];
        let bv = BuildVolume::new_from_config(bed, 200.0, vec![extruder1], vec![200.0]);
        assert_eq!(bv.get_extruder_area_count(), 1);
    }

    #[test]
    fn test_build_extruder_volume_default() {
        // Test BuildExtruderVolume default constructor
        let ev = BuildExtruderVolume::new();
        assert!(!ev.same_with_bed);
        assert_eq!(ev.volume_type, Type::Invalid);
    }

    #[test]
    fn test_volume_state_bbox_rectangle() {
        // BuildVolume.cpp:404-415
        let area = vec![
            Vec2d::new(0.0, 0.0),
            Vec2d::new(200.0, 0.0),
            Vec2d::new(200.0, 200.0),
            Vec2d::new(0.0, 200.0),
        ];
        let bv = BuildVolume::new_from_config(area, 200.0, Vec::new(), Vec::new());
        // Box fully inside the bed.
        let inside_box = BoundingBoxf3::new_from_points(
            Vec3d::new(10.0, 10.0, 10.0),
            Vec3d::new(50.0, 50.0, 50.0),
        );
        assert_eq!(bv.volume_state_bbox(&inside_box, true), ObjectState::Inside);
    }
}
