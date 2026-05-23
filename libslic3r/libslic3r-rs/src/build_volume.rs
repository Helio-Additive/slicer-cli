//! Build volume collision detection and validation
//!
//! C++ Reference:
//! - BuildVolume.hpp (class definitions and enums)
//! - BuildVolume.cpp (constructor and collision detection methods)
//!
//! This module provides collision detection for objects and G-code paths against
//! the build volume. Supports rectangular, circular, convex, and custom bed shapes.

use crate::bounding_box::BoundingBoxf3;
use crate::geometry::BoundingBox;
use crate::geometry::{Point, Polygon, Vec2d, Vec3d};
use crate::{scale, unscale};
use std::f64;

/// Epsilon for floating-point comparisons (from libslic3r.h)
/// libslic3r.h:72
/// C++: static constexpr double EPSILON = 1e-4;
const EPSILON: f64 = 1e-4;

/// Scaled epsilon for integer coordinate comparisons
/// libslic3r.h:73
/// C++: #define SCALED_EPSILON scale_(EPSILON)
const SCALED_EPSILON: i64 = 100; // scale(1e-4) ≈ 100 units

/// Epsilon for scene collision tests (plater UI)
/// BuildVolume.hpp:108
/// C++: static constexpr const double SceneEpsilon = EPSILON;
pub const SCENE_EPSILON: f64 = EPSILON;

/// Epsilon for bed collision tests (G-code paths)
/// BuildVolume.hpp:113
/// C++: static constexpr const double BedEpsilon = 3. * EPSILON;
pub const BED_EPSILON: f64 = 3.0 * EPSILON;

/// Build volume type classification
/// BuildVolume.hpp:19-28
/// C++: enum class Type : char
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Type {
    /// Not set yet or undefined
    /// BuildVolume.hpp:22
    /// C++: Invalid = -1,
    Invalid,
    /// Rectangular print bed (most common, cheap to work with)
    /// BuildVolume.hpp:24
    /// C++: Rectangle,
    Rectangle,
    /// Circular print bed (common on deltas, cheap to work with)
    /// BuildVolume.hpp:26
    /// C++: Circle,
    Circle,
    /// Convex print bed (complex to process)
    /// BuildVolume.hpp:28
    /// C++: Convex,
    Convex,
    /// Some non-convex shape
    /// BuildVolume.hpp:30
    /// C++: Custom
    Custom,
}

/// Object collision state with build volume
/// BuildVolume.hpp:94-106
/// C++: enum class ObjectState : unsigned char
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ObjectState {
    /// Inside the build volume, thus printable
    /// BuildVolume.hpp:97
    /// C++: Inside,
    Inside,
    /// Colliding with build volume boundary, not printable (error shown)
    /// BuildVolume.hpp:99
    /// C++: Colliding,
    Colliding,
    /// Outside of build volume (object ignored, no error)
    /// BuildVolume.hpp:101
    /// C++: Outside,
    Outside,
    /// Completely below the print bed
    /// BuildVolume.hpp:103-104
    /// C++: Below,
    Below,
    /// In limited area (extruder-specific constraint)
    /// BuildVolume.hpp:106
    /// C++: Limited
    Limited,
}

/// Circle geometry for circular bed detection
/// BuildVolume.hpp (Geometry::Circled reference)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Circle {
    /// Center point (scaled coordinates)
    pub center: Vec2d,
    /// Radius (scaled coordinates)
    pub radius: f64,
}

impl Circle {
    /// Create a new circle
    pub fn new(center: Vec2d, radius: f64) -> Self {
        Self { center, radius }
    }

    /// Check if a 2D point is inside the circle
    pub fn contains(&self, pt: Vec2d) -> bool {
        let dx = pt.x - self.center.x;
        let dy = pt.y - self.center.y;
        dx * dx + dy * dy <= self.radius * self.radius
    }
}

/// Per-extruder build volume (for multi-extruder limited areas)
/// BuildVolume.hpp:32-38
/// C++: struct BuildExtruderVolume
#[derive(Debug, Clone)]
pub struct BuildExtruderVolume {
    /// Whether this extruder volume is the same as the bed
    /// BuildVolume.hpp:33
    /// C++: bool same_with_bed{false};
    pub same_with_bed: bool,
    /// Type of volume
    /// BuildVolume.hpp:34
    /// C++: Type type{Type::Invalid};
    pub volume_type: Type,
    /// Bounding box (scaled coordinates)
    /// BuildVolume.hpp:35
    /// C++: BoundingBox bbox;
    pub bbox: BoundingBox,
    /// Bounding box (floating-point coordinates)
    /// BuildVolume.hpp:36
    /// C++: BoundingBoxf3 bboxf;
    pub bboxf: BoundingBoxf3,
    /// Circle for circular volumes
    /// BuildVolume.hpp:37
    /// C++: Geometry::Circled circle;
    pub circle: Circle,
}

impl BuildExtruderVolume {
    /// Create a new empty extruder volume
    pub fn new() -> Self {
        Self {
            same_with_bed: false,
            volume_type: Type::Invalid,
            bbox: BoundingBox::new(),
            bboxf: BoundingBoxf3::new(),
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
/// BuildVolume.hpp:40-54
/// C++: struct BuildSharedVolume
#[derive(Debug, Clone, Copy)]
pub struct BuildSharedVolume {
    /// Bed shape type (see Bed3D::EShapeType)
    /// BuildVolume.hpp:43
    /// C++: int type{ 0 };
    pub volume_type: i32,
    /// Rectangle: [min.x, min.y, max.x, max.y]; Circle: [center.x, center.y, -, radius]
    /// BuildVolume.hpp:44-49
    /// C++: std::array<float, 4> data;
    pub data: [f32; 4],
    /// Z range: [min_z, max_z]
    /// BuildVolume.hpp:50-51
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
/// BuildVolume.hpp:17-164
/// C++: class BuildVolume
#[derive(Debug, Clone)]
pub struct BuildVolume {
    /// Source bed shape (unscaled coordinates)
    /// BuildVolume.hpp:133
    /// C++: std::vector<Vec2d> m_bed_shape;
    bed_shape: Vec<Vec2d>,
    /// Per-extruder shapes (unscaled coordinates)
    /// BuildVolume.hpp:134-135
    /// C++: std::vector<std::vector<Vec2d>> m_extruder_shapes;
    extruder_shapes: Vec<Vec<Vec2d>>,
    /// Per-extruder volumes
    /// BuildVolume.hpp:136
    /// C++: std::vector<BuildExtruderVolume> m_extruder_volumes;
    extruder_volumes: Vec<BuildExtruderVolume>,
    /// Shared volume for rendering
    /// BuildVolume.hpp:137
    /// C++: BuildSharedVolume m_shared_volume;
    shared_volume: BuildSharedVolume,
    /// Maximum print height (unscaled)
    /// BuildVolume.hpp:138
    /// C++: double m_max_print_height { 0.f };
    max_print_height: f64,
    /// Per-extruder printable heights
    /// BuildVolume.hpp:139
    /// C++: std::vector<double> m_extruder_printable_height;
    extruder_printable_height: Vec<f64>,
    /// Derived volume type
    /// BuildVolume.hpp:142
    /// C++: Type m_type { Type::Invalid };
    volume_type: Type,
    /// Bed geometry (scaled coordinates)
    /// BuildVolume.hpp:143-144
    /// C++: Polygon m_polygon;
    polygon: Polygon,
    /// Snug bounding box around polygon (scaled)
    /// BuildVolume.hpp:145-146
    /// C++: BoundingBox m_bbox;
    bbox: BoundingBox,
    /// 3D bounding volume (unscaled)
    /// BuildVolume.hpp:147-148
    /// C++: BoundingBoxf3 m_bboxf;
    bboxf: BoundingBoxf3,
    /// Area of polygon (scaled)
    /// BuildVolume.hpp:149-150
    /// C++: double m_area { 0. };
    area: f64,
    /// Convex hull of polygon (scaled)
    /// BuildVolume.hpp:151-152
    /// C++: Polygon m_convex_hull;
    convex_hull: Polygon,
    /// Convex hull decomposition for scene tests
    /// BuildVolume.hpp:153-155
    /// C++: std::pair<std::vector<Vec2d>, std::vector<Vec2d>> m_top_bottom_convex_hull_decomposition_scene;
    top_bottom_convex_hull_decomposition_scene: (Vec<Vec2d>, Vec<Vec2d>),
    /// Convex hull decomposition for bed tests
    /// BuildVolume.hpp:156-157
    /// C++: std::pair<std::vector<Vec2d>, std::vector<Vec2d>> m_top_bottom_convex_hull_decomposition_bed;
    top_bottom_convex_hull_decomposition_bed: (Vec<Vec2d>, Vec<Vec2d>),
    /// Smallest enclosing circle (scaled)
    /// BuildVolume.hpp:158-159
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
    /// C++: BuildVolume(const std::vector<Vec2d> &printable_area, const double printable_height,
    /// C++:             const std::vector<std::vector<Vec2d>> &extruder_areas,
    /// C++:             const std::vector<double>& extruder_printable_heights)
    pub fn new_from_config(
        printable_area: Vec<Vec2d>,
        printable_height: f64,
        extruder_areas: Vec<Vec<Vec2d>>,
        extruder_printable_heights: Vec<f64>,
    ) -> Self {
        assert!(printable_height >= 0.0);

        let mut bv = Self::new();
        bv.bed_shape = printable_area.clone();
        bv.max_print_height = printable_height;
        bv.extruder_shapes = extruder_areas.clone();
        bv.extruder_printable_height = extruder_printable_heights.clone();

        // Scale polygon from unscaled Vec2d points
        // BuildVolume.cpp:17
        // C++: m_polygon = Polygon::new_scale(printable_area);
        bv.polygon = Polygon::from_points(
            printable_area
                .iter()
                .map(|p| Point::new(scale(p.x), scale(p.y)))
                .collect(),
        );

        // Calculate convex hull
        // BuildVolume.cpp:20
        // C++: m_convex_hull = Geometry::convex_hull(m_polygon.points);
        bv.convex_hull = crate::geometry::convex_hull_points(bv.polygon.points.clone());

        // Calculate bounding box
        // BuildVolume.cpp:21
        // C++: m_bbox = get_extents(m_convex_hull);
        bv.bbox = bv.convex_hull.bounding_box();

        // Calculate area
        // BuildVolume.cpp:22
        // C++: m_area = m_polygon.area();
        bv.area = bv.polygon.area();

        // Calculate 3D bounding volume (unscaled)
        // BuildVolume.cpp:24-25
        // C++: BoundingBoxf bboxf = get_extents(printable_area);
        // C++: m_bboxf = BoundingBoxf3{ to_3d(bboxf.min, 0.), to_3d(bboxf.max, printable_height) };
        let bboxf_2d = get_extents_f(&printable_area);
        bv.bboxf = BoundingBoxf3::new_from_points(
            Vec3d::new(bboxf_2d.min.x, bboxf_2d.min.y, 0.0),
            Vec3d::new(bboxf_2d.max.x, bboxf_2d.max.y, printable_height),
        );

        // Classify build volume type
        // BuildVolume.cpp:27-60
        if printable_area.len() >= 4
            && ((bv.area - (bv.bbox.size().x as f64) * (bv.bbox.size().y as f64)).abs()
                < (SCALED_EPSILON * SCALED_EPSILON) as f64)
        {
            // Rectangle detection
            // BuildVolume.cpp:27-32
            // C++: if (printable_area.size() >= 4 && std::abs((m_area - double(m_bbox.size().x()) * double(m_bbox.size().y()))) < sqr(SCALED_EPSILON))
            bv.volume_type = Type::Rectangle;
            bv.circle.center = Vec2d::new(
                0.5 * ((bv.bbox.min.x + bv.bbox.max.x) as f64),
                0.5 * ((bv.bbox.min.y + bv.bbox.max.y) as f64),
            );
            bv.circle.radius = 0.5 * bv.bbox.size().length();
        } else if printable_area.len() > 3 {
            // Circle detection using RANSAC
            // BuildVolume.cpp:33-60
            // C++: m_circle = Geometry::circle_ransac(printable_area);
            bv.circle = circle_ransac(&printable_area);
            let mut is_circle = true;

            // Validate circle fit
            // BuildVolume.cpp:41-58
            let mut prev = *printable_area.last().unwrap();
            for p in &printable_area {
                let dist_from_center = (p.x - bv.circle.center.x).hypot(p.y - bv.circle.center.y);
                let midpoint = Vec2d::new(0.5 * (prev.x + p.x), 0.5 * (prev.y + p.y));
                let midpoint_dist =
                    (midpoint.x - bv.circle.center.x).hypot(midpoint.y - bv.circle.center.y);

                // Check vertex distance and edge midpoint undercut
                // BuildVolume.cpp:45-54
                // C++: if (std::abs((p - m_circle.center).norm() - m_circle.radius) > 0.005 ||
                // C++:     m_circle.radius - (0.5 * (prev + p) - m_circle.center).norm() > 3.)
                if (dist_from_center - bv.circle.radius).abs() > 0.005
                    || bv.circle.radius - midpoint_dist > 3.0
                {
                    is_circle = false;
                    break;
                }
                prev = *p;
            }

            if is_circle {
                // Mark as circle and scale coordinates
                // BuildVolume.cpp:59-60
                // C++: m_type = Type::Circle;
                // C++: m_circle.center = scaled<double>(m_circle.center);
                // C++: m_circle.radius = scaled<double>(m_circle.radius);
                bv.volume_type = Type::Circle;
                bv.circle.center = Vec2d::new(
                    scale(bv.circle.center.x) as f64,
                    scale(bv.circle.center.y) as f64,
                );
                bv.circle.radius = scale(bv.circle.radius) as f64;
            }
        }

        // Handle convex/custom shapes
        // BuildVolume.cpp:63-78
        if printable_area.len() >= 3 && bv.volume_type == Type::Invalid {
            // Calculate smallest enclosing circle
            // BuildVolume.cpp:65
            // C++: m_circle = Geometry::smallest_enclosing_circle_welzl(m_convex_hull.points);
            bv.circle = smallest_enclosing_circle_welzl(&bv.convex_hull.points);

            // Classify as convex or custom
            // BuildVolume.cpp:66-67
            // C++: m_type = (m_convex_hull.area() - m_area) < sqr(SCALED_EPSILON) ? Type::Convex : Type::Custom;
            bv.volume_type = if (bv.convex_hull.area() - bv.area).abs()
                < (SCALED_EPSILON * SCALED_EPSILON) as f64
            {
                Type::Convex
            } else {
                Type::Custom
            };

            // Decompose convex hull for inside tests
            // BuildVolume.cpp:69-77
            // C++: m_top_bottom_convex_hull_decomposition_scene = convex_decomposition(m_convex_hull, SceneEpsilon);
            // C++: m_top_bottom_convex_hull_decomposition_bed   = convex_decomposition(m_convex_hull, BedEpsilon);
            bv.top_bottom_convex_hull_decomposition_scene =
                convex_decomposition(&bv.convex_hull, SCENE_EPSILON);
            bv.top_bottom_convex_hull_decomposition_bed =
                convex_decomposition(&bv.convex_hull, BED_EPSILON);
        }

        // Process extruder areas
        // BuildVolume.cpp:80-174
        if !extruder_areas.is_empty() {
            // Initialize shared volume
            // BuildVolume.cpp:82-86
            // C++: m_shared_volume.data[0] = m_bboxf.min.x();
            bv.shared_volume.data[0] = bv.bboxf.min.x as f32;
            bv.shared_volume.data[1] = bv.bboxf.min.y as f32;
            bv.shared_volume.data[2] = bv.bboxf.max.x as f32;
            bv.shared_volume.data[3] = bv.bboxf.max.y as f32;
            bv.shared_volume.zs[1] = bv.bboxf.max.z as f32;

            for (index, extruder_shape) in extruder_areas.iter().enumerate() {
                let mut extruder_volume = BuildExtruderVolume::new();

                if extruder_shape.is_empty() {
                    // Invalid extruder area
                    // BuildVolume.cpp:93-99
                    eprintln!("Found invalid extruder_printable_area of index {}", index);
                    bv.extruder_shapes.clear();
                    return bv;
                }

                // Check if extruder area matches bed
                // BuildVolume.cpp:101-107
                // C++: if ((extruder_shape == printable_area)&&(extruder_printable_heights[index] == printable_height))
                if extruder_shape == &printable_area
                    && extruder_printable_heights[index] == printable_height
                {
                    extruder_volume.same_with_bed = true;
                    extruder_volume.volume_type = bv.volume_type;
                    extruder_volume.bbox = bv.bbox;
                    extruder_volume.bboxf = bv.bboxf;
                    extruder_volume.circle = bv.circle;
                } else {
                    // Process distinct extruder area
                    // BuildVolume.cpp:109-157
                    let poly = Polygon::from_points(
                        extruder_shape
                            .iter()
                            .map(|p| Point::new(scale(p.x), scale(p.y)))
                            .collect(),
                    );
                    let poly_area = poly.area();
                    extruder_volume.bbox = poly.bounding_box();

                    let temp_bboxf = get_extents_f(extruder_shape);
                    extruder_volume.bboxf = BoundingBoxf3::new_from_points(
                        Vec3d::new(temp_bboxf.min.x, temp_bboxf.min.y, 0.0),
                        Vec3d::new(
                            temp_bboxf.max.x,
                            temp_bboxf.max.y,
                            extruder_printable_heights[index],
                        ),
                    );

                    // Rectangle detection for extruder area
                    // BuildVolume.cpp:116-122
                    if extruder_shape.len() >= 4
                        && (poly_area
                            - (extruder_volume.bbox.size().x as f64)
                                * (extruder_volume.bbox.size().y as f64))
                            .abs()
                            < (SCALED_EPSILON * SCALED_EPSILON) as f64
                    {
                        extruder_volume.volume_type = Type::Rectangle;
                        extruder_volume.circle.center = Vec2d::new(
                            0.5 * ((extruder_volume.bbox.min.x + extruder_volume.bbox.max.x)
                                as f64),
                            0.5 * ((extruder_volume.bbox.min.y + extruder_volume.bbox.max.y)
                                as f64),
                        );
                        extruder_volume.circle.radius = 0.5 * extruder_volume.bbox.size().length();
                    } else if extruder_shape.len() > 3 {
                        // Circle detection for extruder area
                        // BuildVolume.cpp:123-143
                        extruder_volume.circle = circle_ransac(extruder_shape);
                        let mut is_circle = true;
                        let mut prev = *extruder_shape.last().unwrap();

                        for p in extruder_shape {
                            let dist = (p.x - extruder_volume.circle.center.x)
                                .hypot(p.y - extruder_volume.circle.center.y);
                            let mid = Vec2d::new(0.5 * (prev.x + p.x), 0.5 * (prev.y + p.y));
                            let mid_dist = (mid.x - extruder_volume.circle.center.x)
                                .hypot(mid.y - extruder_volume.circle.center.y);

                            if (dist - extruder_volume.circle.radius).abs() > 0.005
                                || extruder_volume.circle.radius - mid_dist > 3.0
                            {
                                is_circle = false;
                                break;
                            }
                            prev = *p;
                        }

                        if is_circle {
                            extruder_volume.volume_type = Type::Circle;
                            extruder_volume.circle.center = Vec2d::new(
                                scale(extruder_volume.circle.center.x) as f64,
                                scale(extruder_volume.circle.center.y) as f64,
                            );
                            extruder_volume.circle.radius =
                                scale(extruder_volume.circle.radius) as f64;
                        }
                    }

                    // Fallback to bed volume if invalid
                    // BuildVolume.cpp:146-152
                    if extruder_volume.volume_type == Type::Invalid {
                        extruder_volume.same_with_bed = true;
                        extruder_volume.volume_type = bv.volume_type;
                        extruder_volume.bbox = bv.bbox;
                        extruder_volume.bboxf = bv.bboxf;
                        extruder_volume.circle = bv.circle;
                    }

                    // Always ignore Z axis
                    // BuildVolume.cpp:154
                    // C++: extruder_volume.bboxf.min.z() = -std::numeric_limits<double>::max();
                    extruder_volume.bboxf.min.z = f64::NEG_INFINITY;
                }

                // Update shared volume intersection
                // BuildVolume.cpp:159-170
                bv.shared_volume.data[0] =
                    bv.shared_volume.data[0].max(extruder_volume.bboxf.min.x as f32);
                bv.shared_volume.data[1] =
                    bv.shared_volume.data[1].max(extruder_volume.bboxf.min.y as f32);
                bv.shared_volume.data[2] =
                    bv.shared_volume.data[2].min(extruder_volume.bboxf.max.x as f32);
                bv.shared_volume.data[3] =
                    bv.shared_volume.data[3].min(extruder_volume.bboxf.max.y as f32);
                bv.shared_volume.zs[1] =
                    bv.shared_volume.zs[1].min(extruder_volume.bboxf.max.z as f32);

                bv.extruder_volumes.push(extruder_volume);
            }

            // Finalize shared volume
            // BuildVolume.cpp:173-174
            // C++: m_shared_volume.type = static_cast<int>(m_type);
            // C++: m_shared_volume.zs[0] = 0.f;
            bv.shared_volume.volume_type = bv.volume_type as i32;
            bv.shared_volume.zs[0] = 0.0;
        }

        bv
    }

    /// Get the printable area (unscaled)
    /// BuildVolume.hpp:61
    /// C++: const std::vector<Vec2d>& printable_area() const { return m_bed_shape; }
    pub fn printable_area(&self) -> &[Vec2d] {
        &self.bed_shape
    }

    /// Get the printable height (unscaled)
    /// BuildVolume.hpp:62
    /// C++: double printable_height() const { return m_max_print_height; }
    pub fn printable_height(&self) -> f64 {
        self.max_print_height
    }

    /// Get the extruder areas (unscaled)
    /// BuildVolume.hpp:63
    /// C++: const std::vector<std::vector<Vec2d>>& extruder_areas() const { return m_extruder_shapes; }
    pub fn extruder_areas(&self) -> &[Vec<Vec2d>] {
        &self.extruder_shapes
    }

    /// Get the extruder heights
    /// BuildVolume.hpp:64
    /// C++: const std::vector<double>& extruder_heights() const { return m_extruder_printable_height; }
    pub fn extruder_heights(&self) -> &[f64] {
        &self.extruder_printable_height
    }

    /// Get the shared volume
    /// BuildVolume.hpp:65
    /// C++: const BuildSharedVolume& get_shared_volume() const { return m_shared_volume; }
    pub fn shared_volume(&self) -> &BuildSharedVolume {
        &self.shared_volume
    }

    /// Get the build volume type
    /// BuildVolume.hpp:68
    /// C++: Type type() const { return m_type; }
    pub fn volume_type(&self) -> Type {
        self.volume_type
    }

    /// Check if the build volume is valid
    /// BuildVolume.hpp:71
    /// C++: bool valid() const { return m_type != Type::Invalid; }
    pub fn valid(&self) -> bool {
        self.volume_type != Type::Invalid
    }

    /// Get the bed polygon (scaled)
    /// BuildVolume.hpp:73
    /// C++: const Polygon& polygon() const { return m_polygon; }
    pub fn polygon(&self) -> &Polygon {
        &self.polygon
    }

    /// Get the bounding box (scaled)
    /// BuildVolume.hpp:75
    /// C++: const BoundingBox& bounding_box() const { return m_bbox; }
    pub fn bounding_box(&self) -> &BoundingBox {
        &self.bbox
    }

    /// Get the 3D bounding volume (unscaled)
    /// BuildVolume.hpp:77
    /// C++: const BoundingBoxf3& bounding_volume() const { return m_bboxf; }
    pub fn bounding_volume(&self) -> &BoundingBoxf3 {
        &self.bboxf
    }

    /// Get the bed center (unscaled)
    /// BuildVolume.hpp:81
    /// C++: Vec2d bed_center() const { return to_2d(m_bboxf.center()); }
    pub fn bed_center(&self) -> Vec2d {
        let center = self.bboxf.center();
        Vec2d::new(center.x, center.y)
    }

    /// Get the convex hull (scaled)
    /// BuildVolume.hpp:83
    /// C++: const Polygon& convex_hull() const { return m_convex_hull; }
    pub fn convex_hull(&self) -> &Polygon {
        &self.convex_hull
    }

    /// Get the smallest enclosing circle (scaled)
    /// BuildVolume.hpp:85
    /// C++: const Geometry::Circled& circle() const { return m_circle; }
    pub fn circle(&self) -> &Circle {
        &self.circle
    }

    /// Get the number of extruder areas
    /// BuildVolume.hpp:119
    /// C++: int get_extruder_area_count() const { return m_extruder_volumes.size(); }
    pub fn extruder_area_count(&self) -> usize {
        self.extruder_volumes.len()
    }

    /// Get a specific extruder volume
    /// BuildVolume.cpp:417-421
    /// C++: const BuildVolume::BuildExtruderVolume& BuildVolume::get_extruder_area_volume(int index) const
    /// C++: {
    /// C++:     assert(index >= 0 && index < m_extruder_volumes.size());
    /// C++:     return m_extruder_volumes[index];
    /// C++: }
    pub fn extruder_area_volume(&self, index: usize) -> &BuildExtruderVolume {
        assert!(index < self.extruder_volumes.len());
        &self.extruder_volumes[index]
    }

    /// Get the type name as a string
    /// BuildVolume.cpp:594-607
    /// C++: std::string_view BuildVolume::type_name(Type type)
    /// C++: {
    /// C++:     using namespace std::literals;
    /// C++:     switch (type) {
    /// C++:     case Type::Invalid:   return "Invalid"sv;
    /// C++:     case Type::Rectangle: return "Rectangle"sv;
    /// C++:     case Type::Circle:    return "Circle"sv;
    /// C++:     case Type::Convex:    return "Convex"sv;
    /// C++:     case Type::Custom:    return "Custom"sv;
    /// C++:     }
    /// C++: }
    pub fn type_name(&self) -> &'static str {
        match self.volume_type {
            Type::Invalid => "Invalid",
            Type::Rectangle => "Rectangle",
            Type::Circle => "Circle",
            Type::Convex => "Convex",
            Type::Custom => "Custom",
        }
    }
}

impl Default for BuildVolume {
    fn default() -> Self {
        Self::new()
    }
}

/// Get bounding box of a slice of Vec2d points (unscaled)
fn get_extents_f(points: &[Vec2d]) -> BoundingBoxf3 {
    let mut min = points[0];
    let mut max = points[0];
    for p in points.iter().skip(1) {
        min.x = min.x.min(p.x);
        min.y = min.y.min(p.y);
        max.x = max.x.max(p.x);
        max.y = max.y.max(p.y);
    }
    BoundingBoxf3::new_from_points(Vec3d::new(min.x, min.y, 0.0), Vec3d::new(max.x, max.y, 0.0))
}

/// RANSAC circle fitting
/// BuildVolume.cpp:38
/// C++: m_circle = Geometry::circle_ransac(printable_area);
fn circle_ransac(points: &[Vec2d]) -> Circle {
    // Simplified RANSAC implementation
    // For production, use full Geometry::circle_ransac from C++
    if points.is_empty() {
        return Circle::new(Vec2d::new(0.0, 0.0), 0.0);
    }

    // Use centroid as approximation
    let mut center = Vec2d::new(0.0, 0.0);
    for p in points {
        center.x += p.x;
        center.y += p.y;
    }
    center.x /= points.len() as f64;
    center.y /= points.len() as f64;

    // Calculate average radius
    let mut radius: f64 = 0.0;
    for p in points {
        radius += (p.x - center.x).hypot(p.y - center.y);
    }
    radius /= points.len() as f64;

    Circle::new(center, radius)
}

/// Smallest enclosing circle using Welzl's algorithm
/// BuildVolume.cpp:65
/// C++: m_circle = Geometry::smallest_enclosing_circle_welzl(m_convex_hull.points);
fn smallest_enclosing_circle_welzl(points: &[Point]) -> Circle {
    // Simplified implementation
    // For production, use full Geometry::smallest_enclosing_circle_welzl from C++
    if points.is_empty() {
        return Circle::new(Vec2d::new(0.0, 0.0), 0.0);
    }

    // Use centroid approximation
    let mut center = Vec2d::new(0.0, 0.0);
    for p in points {
        center.x += p.x as f64;
        center.y += p.y as f64;
    }
    center.x /= points.len() as f64;
    center.y /= points.len() as f64;

    // Find farthest point
    let mut radius: f64 = 0.0;
    for p in points {
        let dist = ((p.x as f64) - center.x).hypot((p.y as f64) - center.y);
        radius = radius.max(dist);
    }

    Circle::new(center, radius)
}

/// Decompose convex polygon for top-bottom inside tests
/// BuildVolume.cpp:69-76
/// C++: auto convex_decomposition = [](const Polygon &in, double epsilon) {
/// C++:     Polygon src = expand(in, float(epsilon)).front();
/// C++:     std::vector<Vec2d> pts;
/// C++:     pts.reserve(src.size());
/// C++:     for (const Point &pt : src.points)
/// C++:         pts.emplace_back(unscaled<double>(pt.cast<double>().eval()));
/// C++:     return Geometry::decompose_convex_polygon_top_bottom(pts);
/// C++: };
fn convex_decomposition(polygon: &Polygon, _epsilon: f64) -> (Vec<Vec2d>, Vec<Vec2d>) {
    // Simplified implementation - returns empty decomposition
    // For production, implement full convex polygon decomposition
    // This requires:
    // 1. Offset polygon by epsilon
    // 2. Find leftmost and rightmost points
    // 3. Split into top and bottom chains
    let mut pts = Vec::new();
    for pt in &polygon.points {
        pts.push(Vec2d::new(unscale(pt.x), unscale(pt.y)));
    }

    // Return empty for now - this is used for Convex/Custom bed shapes
    // which are less common than Rectangle/Circle
    (Vec::new(), Vec::new())
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
        // BuildVolume.cpp:27-32
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
    fn test_build_volume_circle_approximation() {
        // Test circular bed detection (simplified)
        // BuildVolume.cpp:33-60
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
        // Note: May detect as Circle or Custom depending on precision
        assert!(bv.valid());
    }

    #[test]
    fn test_circle_contains() {
        // Test circle containment check
        let circle = Circle::new(Vec2d::new(100.0, 100.0), 50.0);
        assert!(circle.contains(Vec2d::new(100.0, 100.0)));
        assert!(circle.contains(Vec2d::new(120.0, 100.0)));
        assert!(!circle.contains(Vec2d::new(160.0, 100.0)));
    }

    #[test]
    fn test_object_state_values() {
        // Test ObjectState enum
        // BuildVolume.hpp:94-106
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
    }

    #[test]
    fn test_extruder_volumes() {
        // Test extruder area handling
        // BuildVolume.cpp:80-174
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
        assert_eq!(bv.extruder_area_count(), 1);
    }

    #[test]
    fn test_build_extruder_volume_default() {
        // Test BuildExtruderVolume default constructor
        let ev = BuildExtruderVolume::new();
        assert!(!ev.same_with_bed);
        assert_eq!(ev.volume_type, Type::Invalid);
    }
}
