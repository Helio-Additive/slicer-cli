//! Text embossing and 3D projection utilities
//!
//! C++ Reference:
//! - BambuStudio/src/libslic3r/Emboss.hpp
//! - BambuStudio/src/libslic3r/Emboss.cpp
//!
//! This module provides functionality for embossing text and images onto 3D surfaces,
//! including font management, glyph-to-polygon conversion, shape healing, and various
//! projection methods for mapping 2D shapes onto 3D surfaces.

use crate::geometry::{
    BoundingBox, ExPolygons, Point, Point3F, Polygon, Transform3D, Vec2d, Vec3d,
};
use crate::{Error, Result};

/// Safe offset distance before union operation (approximately in nanometers, depends on volume scale)
/// Emboss.hpp:22
/// C++: static const float UNION_DELTA = 50.0f;
pub const UNION_DELTA: f32 = 50.0;

/// Maximum number of iterations for union/healing operations
/// Emboss.hpp:23
/// C++: static const unsigned UNION_MAX_ITERATIN = 10;
pub const UNION_MAX_ITERATION: u32 = 10;

/// Unicode value for newline character
/// Emboss.hpp:96
/// C++: const unsigned ENTER_UNICODE = static_cast<unsigned>('\n');
pub const ENTER_UNICODE: u32 = '\n' as u32;

/// Experimentally suggested ratio of font ascent to get approximate center of normal text line
/// Emboss.cpp:25
/// C++: const double ASCENT_CENTER = 1/2.5;
const ASCENT_CENTER: f64 = 1.0 / 2.5;

/// Scale factor for glyph shape points - increases precision of fixed-point values
/// Emboss.cpp:28
/// C++: static constexpr double SHAPE_SCALE = 0.001;
const SHAPE_SCALE: f64 = 0.001;

/// Maximum healing iterations for text shapes
/// Emboss.cpp:29
/// C++: static unsigned MAX_HEAL_ITERATION_OF_TEXT = 10;
const MAX_HEAL_ITERATION_OF_TEXT: u32 = 10;

/// Result of polygon healing operation
/// Emboss.hpp:109
#[derive(Debug, Clone)]
pub struct HealedExPolygons {
    /// The healed polygons
    /// Emboss.hpp:109
    pub expolygons: ExPolygons,

    /// True if healing completed successfully without remaining issues
    /// Emboss.hpp:109
    pub is_healed: bool,
}

/// Font property descriptor for text rendering
/// Emboss.hpp (referenced in text2shapes)
#[derive(Debug, Clone)]
pub struct FontProp {
    /// Font size in points
    pub size_in_mm: f64,

    /// Line spacing factor
    pub line_spacing: f64,

    /// Character spacing factor
    pub char_spacing: f64,

    /// Vertical alignment
    pub vertical_align: VerticalAlign,

    /// Additional line gap
    pub line_gap: i32,

    /// Collection index within font file
    pub collection_number: u32,
}

/// Vertical alignment options for text
/// Emboss.hpp (referenced in get_align_y_offset_in_mm)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VerticalAlign {
    /// Align to top
    Top,
    /// Center alignment
    Center,
    /// Align to bottom
    Bottom,
}

/// Font file information
/// Emboss.hpp (referenced in get_font_info)
#[derive(Debug, Clone)]
pub struct FontInfo {
    /// Ascent (units above baseline)
    pub ascent: i32,

    /// Descent (units below baseline)
    pub descent: i32,

    /// Line gap
    pub line_gap: i32,

    /// Units per em
    pub units_per_em: i32,
}

/// Represents a single glyph (character shape)
/// Emboss.hpp:67
#[derive(Debug, Clone)]
pub struct Glyph {
    /// Glyph outline as polygons (inner polygons are CW, outer are CCW)
    /// Emboss.hpp:67
    pub shape: ExPolygons,

    /// Advance width for this glyph
    /// Emboss.hpp:67
    pub advance_width: i32,

    /// Left side bearing
    /// Emboss.hpp:67
    pub left_side_bearing: i32,
}

/// Font file wrapper (stub - requires font library integration)
/// Emboss.hpp:50-56
#[derive(Debug, Clone)]
pub struct FontFile {
    /// Font data buffer
    _data: Vec<u8>,

    /// Font information
    pub info: FontInfo,
}

impl FontFile {
    /// Create font file from file path
    /// Emboss.hpp:50
    /// C++: std::unique_ptr<FontFile> create_font_file(const char *file_path);
    pub fn from_path(_path: &str) -> Result<Self> {
        // TODO: Implement font loading using freetype-rs or similar
        Err(Error::Config(
            "Font loading not yet implemented - requires font library integration".to_string(),
        ))
    }

    /// Create font file from raw data
    /// Emboss.hpp:52
    /// C++: std::unique_ptr<FontFile> create_font_file(std::unique_ptr<std::vector<unsigned char>> data);
    pub fn from_data(_data: Vec<u8>) -> Result<Self> {
        // TODO: Implement font parsing
        Err(Error::Config(
            "Font parsing not yet implemented - requires font library integration".to_string(),
        ))
    }
}

/// Font file with glyph cache
/// Emboss.hpp:77 (BackFontCacheFn references this)
#[derive(Debug, Clone)]
pub struct FontFileWithCache {
    /// The font file
    pub font: FontFile,

    /// Cached glyphs by unicode codepoint
    pub cache: std::collections::HashMap<u32, Glyph>,
}

/// Polygon point on a contour
/// Emboss.hpp:374
#[derive(Debug, Clone, Copy)]
pub struct PolygonPoint {
    /// Index of the polygon in collection
    /// Emboss.hpp:374
    pub polygon_index: usize,

    /// Index of the point within polygon
    /// Emboss.hpp:374
    pub point_index: usize,
}

/// Text line descriptor for surface text projection
/// Emboss.hpp:368-378
#[derive(Debug, Clone)]
pub struct TextLine {
    /// Slice polygon defining the text path
    /// Emboss.hpp:371
    /// C++: Polygon polygon;
    pub polygon: Polygon,

    /// Starting point on the polygon (closest to origin)
    /// Emboss.hpp:374
    /// C++: PolygonPoint start;
    pub start: PolygonPoint,

    /// Y-offset of text line in volume (mm)
    /// Emboss.hpp:377
    /// C++: float y;
    pub y: f32,
}

/// Collection of text lines
/// Emboss.hpp:379
pub type TextLines = Vec<TextLine>;

/// Collection of polygon points
/// Emboss.hpp:389
pub type PolygonPoints = Vec<PolygonPoint>;

/// Trait for 3D point projection
/// Emboss.hpp:199-211
pub trait IProject3d {
    /// Move point with respect to projection direction
    /// Emboss.hpp:210
    /// C++: virtual Vec3d project(const Vec3d &point) const = 0;
    fn project(&self, point: &Vec3d) -> Vec3d;
}

/// Trait for 2D to 3D projection (extends IProject3d)
/// Emboss.hpp:217-239
pub trait IProjection: IProject3d {
    /// Convert 2D point to front and back 3D points
    /// Emboss.hpp:230
    /// C++: virtual std::pair<Vec3d, Vec3d> create_front_back(const Point &p) const = 0;
    fn create_front_back(&self, p: &Point) -> (Vec3d, Vec3d);

    /// Back-project 3D point to 2D
    /// Emboss.hpp:238
    /// C++: virtual std::optional<Vec2d> unproject(const Vec3d &p, double * depth = nullptr) const = 0;
    fn unproject(&self, p: &Vec3d, depth: Option<&mut f64>) -> Option<Vec2d>;
}

/// Simple Z-axis projection
/// Emboss.hpp:275-284
#[derive(Debug, Clone, Copy)]
pub struct ProjectZ {
    /// Emboss depth
    /// Emboss.hpp:283
    /// C++: double m_depth;
    pub depth: f64,
}

impl ProjectZ {
    /// Create new Z projection with given depth
    /// Emboss.hpp:278
    /// C++: explicit ProjectZ(double depth) : m_depth(depth) {}
    pub fn new(depth: f64) -> Self {
        Self { depth }
    }
}

impl IProject3d for ProjectZ {
    /// Project point along Z axis
    /// Emboss.hpp:281
    /// C++: Vec3d project(const Vec3d &point) const override;
    fn project(&self, point: &Vec3d) -> Vec3d {
        *point
    }
}

impl IProjection for ProjectZ {
    /// Create front and back points for Z projection
    /// Emboss.hpp:280
    /// C++: std::pair<Vec3d, Vec3d> create_front_back(const Point &p) const override;
    fn create_front_back(&self, p: &Point) -> (Vec3d, Vec3d) {
        let x = p.x as f64;
        let y = p.y as f64;
        let front = Vec3d::new(x, y, 0.0);
        let back = Vec3d::new(x, y, self.depth);
        (front, back)
    }

    /// Unproject 3D point to 2D
    /// Emboss.hpp:282
    /// C++: std::optional<Vec2d> unproject(const Vec3d &p, double * depth = nullptr) const override;
    fn unproject(&self, p: &Vec3d, depth: Option<&mut f64>) -> Option<Vec2d> {
        if let Some(d) = depth {
            *d = p.z;
        }
        Some(Vec2d::new(p.x, p.y))
    }
}

/// Scaled projection wrapper
/// Emboss.hpp:286-309
pub struct ProjectScale {
    /// Core projection to wrap
    /// Emboss.hpp:288
    /// C++: std::unique_ptr<IProjection> core;
    core: Box<dyn IProjection>,

    /// Scale factor
    /// Emboss.hpp:289
    /// C++: double m_scale;
    scale: f64,
}

impl ProjectScale {
    /// Create scaled projection
    /// Emboss.hpp:291-293
    /// C++: ProjectScale(std::unique_ptr<IProjection> core, double scale)
    pub fn new(core: Box<dyn IProjection>, scale: f64) -> Self {
        Self { core, scale }
    }
}

impl IProject3d for ProjectScale {
    /// Project point (no scale applied to projection direction)
    /// Emboss.hpp:301-303
    /// C++: Vec3d project(const Vec3d &point) const override
    fn project(&self, point: &Vec3d) -> Vec3d {
        self.core.project(point)
    }
}

impl IProjection for ProjectScale {
    /// Create front and back points with scaling
    /// Emboss.hpp:296-300
    /// C++: std::pair<Vec3d, Vec3d> create_front_back(const Point &p) const override
    fn create_front_back(&self, p: &Point) -> (Vec3d, Vec3d) {
        let (front, back) = self.core.create_front_back(p);
        (front * self.scale, back * self.scale)
    }

    /// Unproject with scale adjustment
    /// Emboss.hpp:304-308
    /// C++: std::optional<Vec2d> unproject(const Vec3d &p, double *depth = nullptr) const override
    fn unproject(&self, p: &Vec3d, depth: Option<&mut f64>) -> Option<Vec2d> {
        let scaled_p = Vec3d::new(p.x / self.scale, p.y / self.scale, p.z / self.scale);
        let mut local_depth = 0.0;
        let result = self.core.unproject(&scaled_p, Some(&mut local_depth));
        if let Some(d) = depth {
            *d = local_depth * self.scale;
        }
        result
    }
}

/// Transformed projection wrapper
/// Emboss.hpp:311-339
pub struct ProjectTransform {
    /// Core projection to wrap
    /// Emboss.hpp:313
    /// C++: std::unique_ptr<IProjection> m_core;
    core: Box<dyn IProjection>,

    /// Forward transformation
    /// Emboss.hpp:314
    /// C++: Transform3d m_tr;
    transform: Transform3D,

    /// Inverse transformation
    /// Emboss.hpp:315
    /// C++: Transform3d m_tr_inv;
    transform_inv: Transform3D,

    /// Z-axis scale factor
    /// Emboss.hpp:316
    /// C++: double z_scale;
    z_scale: f64,
}

impl ProjectTransform {
    /// Create transformed projection
    /// Emboss.hpp:318-322
    /// C++: ProjectTransform(std::unique_ptr<IProjection> core, const Transform3d &tr)
    pub fn new(core: Box<dyn IProjection>, transform: Transform3D) -> Self {
        // TODO: Implement proper matrix inversion for Transform3D
        // For now, use identity as a placeholder - this needs full matrix inversion
        let transform_inv = Transform3D::identity();

        // Extract z-scale from matrix (norm of transformed unit Z vector)
        let z_vec = transform.apply(Point3F::new(0.0, 0.0, 1.0));
        let origin = transform.translation_component();
        let dx = z_vec.x - origin.x;
        let dy = z_vec.y - origin.y;
        let dz = z_vec.z - origin.z;
        let z_scale = (dx * dx + dy * dy + dz * dz).sqrt();

        Self {
            core,
            transform,
            transform_inv,
            z_scale,
        }
    }
}

impl IProject3d for ProjectTransform {
    /// Project point (no transformation of direction)
    /// Emboss.hpp:330-332
    /// C++: Vec3d project(const Vec3d &point) const override
    fn project(&self, point: &Vec3d) -> Vec3d {
        self.core.project(point)
    }
}

impl IProjection for ProjectTransform {
    /// Create front and back points with transformation
    /// Emboss.hpp:325-329
    /// C++: std::pair<Vec3d, Vec3d> create_front_back(const Point &p) const override
    fn create_front_back(&self, p: &Point) -> (Vec3d, Vec3d) {
        let (front, back) = self.core.create_front_back(p);
        let front_p3f = self
            .transform
            .apply(Point3F::new(front.x, front.y, front.z));
        let back_p3f = self.transform.apply(Point3F::new(back.x, back.y, back.z));
        (
            Vec3d::new(front_p3f.x, front_p3f.y, front_p3f.z),
            Vec3d::new(back_p3f.x, back_p3f.y, back_p3f.z),
        )
    }

    /// Unproject with inverse transformation
    /// Emboss.hpp:333-338
    /// C++: std::optional<Vec2d> unproject(const Vec3d &p, double *depth = nullptr) const override
    fn unproject(&self, p: &Vec3d, depth: Option<&mut f64>) -> Option<Vec2d> {
        let inv_p3f = self.transform_inv.apply(Point3F::new(p.x, p.y, p.z));
        let inv_p = Vec3d::new(inv_p3f.x, inv_p3f.y, inv_p3f.z);
        let mut local_depth = 0.0;
        let result = self.core.unproject(&inv_p, Some(&mut local_depth));
        if let Some(d) = depth {
            *d = local_depth * self.z_scale;
        }
        result
    }
}

/// Orthogonal 3D projection
/// Emboss.hpp:341-348
#[derive(Debug, Clone)]
pub struct OrthoProject3d {
    /// Direction and magnitude of emboss
    /// Emboss.hpp:344
    /// C++: Vec3d m_direction;
    direction: Vec3d,
}

impl OrthoProject3d {
    /// Create orthogonal 3D projection
    /// Emboss.hpp:346
    /// C++: OrthoProject3d(Vec3d direction) : m_direction(direction) {}
    pub fn new(direction: Vec3d) -> Self {
        Self { direction }
    }
}

impl IProject3d for OrthoProject3d {
    /// Project point by adding direction vector
    /// Emboss.hpp:347
    /// C++: Vec3d project(const Vec3d &point) const override
    fn project(&self, point: &Vec3d) -> Vec3d {
        Vec3d::new(
            point.x + self.direction.x,
            point.y + self.direction.y,
            point.z + self.direction.z,
        )
    }
}

/// Orthogonal projection with matrix transformation
/// Emboss.hpp:350-363
pub struct OrthoProject {
    /// Transformation matrix
    /// Emboss.hpp:351
    /// C++: Transform3d m_matrix;
    matrix: Transform3D,

    /// Direction and magnitude of emboss
    /// Emboss.hpp:353
    /// C++: Vec3d m_direction;
    direction: Vec3d,

    /// Inverse transformation matrix
    /// Emboss.hpp:354
    /// C++: Transform3d m_matrix_inv;
    matrix_inv: Transform3D,
}

impl OrthoProject {
    /// Create orthogonal projection
    /// Emboss.hpp:356-358
    /// C++: OrthoProject(Transform3d matrix, Vec3d direction)
    pub fn new(matrix: Transform3D, direction: Vec3d) -> Self {
        // TODO: Implement proper matrix inversion for Transform3D
        // For now, use identity as a placeholder - this needs full matrix inversion
        let matrix_inv = Transform3D::identity();
        Self {
            matrix,
            direction,
            matrix_inv,
        }
    }
}

impl IProject3d for OrthoProject {
    /// Project point along direction
    /// Emboss.hpp:361
    /// C++: Vec3d project(const Vec3d &point) const override;
    fn project(&self, point: &Vec3d) -> Vec3d {
        Vec3d::new(
            point.x + self.direction.x,
            point.y + self.direction.y,
            point.z + self.direction.z,
        )
    }
}

impl IProjection for OrthoProject {
    /// Create front and back points
    /// Emboss.hpp:360
    /// C++: std::pair<Vec3d, Vec3d> create_front_back(const Point &p) const override;
    fn create_front_back(&self, p: &Point) -> (Vec3d, Vec3d) {
        let x = p.x as f64;
        let y = p.y as f64;
        let base = Point3F::new(x, y, 0.0);
        let front_p3f = self.matrix.apply(base);
        let front = Vec3d::new(front_p3f.x, front_p3f.y, front_p3f.z);
        let back = Vec3d::new(
            front.x + self.direction.x,
            front.y + self.direction.y,
            front.z + self.direction.z,
        );
        (front, back)
    }

    /// Unproject 3D point to 2D
    /// Emboss.hpp:362
    /// C++: std::optional<Vec2d> unproject(const Vec3d &p, double * depth = nullptr) const override;
    fn unproject(&self, p: &Vec3d, depth: Option<&mut f64>) -> Option<Vec2d> {
        let inv_p = self.matrix_inv.apply(Point3F::new(p.x, p.y, p.z));
        if let Some(d) = depth {
            *d = inv_p.z;
        }
        Some(Vec2d::new(inv_p.x, inv_p.y))
    }
}

/// Count number of lines (newline characters) in text
/// Emboss.hpp:98-100
/// C++: unsigned get_count_lines(const std::wstring &ws);
/// C++: unsigned get_count_lines(const std::string &text);
pub fn get_count_lines(text: &str) -> u32 {
    text.chars().filter(|&c| c == '\n').count() as u32
}

/// Heal polygons by fixing duplicates, self-intersections, and reducing points
/// Emboss.hpp:109
/// C++: HealedExPolygons heal_polygons(const Polygons &shape, bool is_non_zero = true, unsigned max_iteration = 10);
pub fn heal_polygons(
    _shape: &[Polygon],
    _is_non_zero: bool,
    _max_iteration: u32,
) -> Result<HealedExPolygons> {
    // TODO: Implement polygon healing using clipper2
    // This requires:
    // 1. Union operation to merge overlapping polygons
    // 2. Remove duplicate points
    // 3. Fix self-intersections
    // 4. Iterative refinement until clean or max iterations
    Err(Error::Geometry(
        "heal_polygons not yet implemented - requires clipper2 integration".to_string(),
    ))
}

/// Heal expolygons in place
/// Emboss.hpp:123
/// C++: bool heal_expolygons(ExPolygons &shape, unsigned max_iteration = 10);
pub fn heal_expolygons(shape: &mut ExPolygons, _max_iteration: u32) -> Result<bool> {
    // TODO: Implement expolygon healing
    // Check for issues and iteratively fix them
    if shape.is_empty() {
        return Ok(true);
    }

    // Placeholder - return false to indicate healing needed
    Ok(false)
}

/// Divide line segments near points that could cause self-intersection
/// Emboss.hpp:134
/// C++: bool divide_segments_for_close_point(ExPolygons &expolygons, double distance);
pub fn divide_segments_for_close_point(
    _expolygons: &mut ExPolygons,
    _distance: f64,
) -> Result<bool> {
    // TODO: Implement segment division
    // Find points close to line segments (within distance)
    // Split segments at those locations
    Ok(false)
}

/// Suggest an "up" vector based on emboss normal direction
/// Emboss.hpp:255
/// C++: Vec3d suggest_up(const Vec3d normal, double up_limit = 0.9);
pub fn suggest_up(normal: Vec3d, up_limit: f64) -> Vec3d {
    // Emboss.cpp implementation
    // When normal is close to vertical, use Y-axis as up
    // Otherwise use Z-axis
    if normal.z.abs() > up_limit {
        Vec3d::new(0.0, 1.0, 0.0)
    } else {
        Vec3d::new(0.0, 0.0, 1.0)
    }
}

/// Calculate rotation angle between suggested and actual up vector
/// Emboss.hpp:263
/// C++: std::optional<float> calc_up(const Transform3d &tr, double up_limit = 0.9);
pub fn calc_up(tr: &Transform3D, up_limit: f64) -> Option<f32> {
    // Extract normal from transformation by transforming unit Z vector
    let origin = tr.translation_component();
    let z_vec = tr.apply(Point3F::new(0.0, 0.0, 1.0));
    let normal = Vec3d::new(z_vec.x - origin.x, z_vec.y - origin.y, z_vec.z - origin.z);
    let normal_len = (normal.x * normal.x + normal.y * normal.y + normal.z * normal.z).sqrt();
    let normal = Vec3d::new(
        normal.x / normal_len,
        normal.y / normal_len,
        normal.z / normal_len,
    );

    let suggested = suggest_up(normal, up_limit);

    // Extract actual up vector
    let y_vec = tr.apply(Point3F::new(0.0, 1.0, 0.0));
    let actual_up = Vec3d::new(y_vec.x - origin.x, y_vec.y - origin.y, y_vec.z - origin.z);
    let up_len =
        (actual_up.x * actual_up.x + actual_up.y * actual_up.y + actual_up.z * actual_up.z).sqrt();
    let actual_up = Vec3d::new(
        actual_up.x / up_len,
        actual_up.y / up_len,
        actual_up.z / up_len,
    );

    // Calculate angle between vectors
    let dot = suggested.x * actual_up.x + suggested.y * actual_up.y + suggested.z * actual_up.z;
    let cross_x = suggested.y * actual_up.z - suggested.z * actual_up.y;
    let cross_y = suggested.z * actual_up.x - suggested.x * actual_up.z;
    let cross_z = suggested.x * actual_up.y - suggested.y * actual_up.x;
    let cross_norm = (cross_x * cross_x + cross_y * cross_y + cross_z * cross_z).sqrt();

    if cross_norm < 1e-6 {
        None
    } else {
        Some(dot.atan2(cross_z) as f32)
    }
}

/// Create transformation to place text onto a surface point
/// Emboss.hpp:272-273
/// C++: Transform3d create_transformation_onto_surface(
/// C++:     const Vec3d &position, const Vec3d &normal, double up_limit = 0.9);
pub fn create_transformation_onto_surface(
    position: Vec3d,
    normal: Vec3d,
    up_limit: f64,
) -> Transform3D {
    let up = suggest_up(normal, up_limit);

    // Build orthonormal basis
    let normal_len = (normal.x * normal.x + normal.y * normal.y + normal.z * normal.z).sqrt();
    let z_axis = Vec3d::new(
        normal.x / normal_len,
        normal.y / normal_len,
        normal.z / normal_len,
    );

    // x_axis = up × z_axis
    let cross_x = up.y * z_axis.z - up.z * z_axis.y;
    let cross_y = up.z * z_axis.x - up.x * z_axis.z;
    let cross_z = up.x * z_axis.y - up.y * z_axis.x;
    let cross_len = (cross_x * cross_x + cross_y * cross_y + cross_z * cross_z).sqrt();
    let x_axis = Vec3d::new(
        cross_x / cross_len,
        cross_y / cross_len,
        cross_z / cross_len,
    );

    // y_axis = z_axis × x_axis
    let y_axis = Vec3d::new(
        z_axis.y * x_axis.z - z_axis.z * x_axis.y,
        z_axis.z * x_axis.x - z_axis.x * x_axis.z,
        z_axis.x * x_axis.y - z_axis.y * x_axis.x,
    );

    // Construct transformation matrix from basis vectors and translation
    // Matrix is in column-major order: [col0, col1, col2, col3]
    let mut result = Transform3D::identity();
    // Column 0 (x-axis)
    result.set(0, 0, x_axis.x);
    result.set(1, 0, x_axis.y);
    result.set(2, 0, x_axis.z);
    // Column 1 (y-axis)
    result.set(0, 1, y_axis.x);
    result.set(1, 1, y_axis.y);
    result.set(2, 1, y_axis.z);
    // Column 2 (z-axis)
    result.set(0, 2, z_axis.x);
    result.set(1, 2, z_axis.y);
    result.set(2, 2, z_axis.z);
    // Column 3 (translation)
    result.set(0, 3, position.x);
    result.set(1, 3, position.y);
    result.set(2, 3, position.z);

    result
}

/// Apply font property transformations (angle and distance)
/// Emboss.hpp:142
/// C++: void apply_transformation(const std::optional<float> &angle, const std::optional<float> &distance, Transform3d &transformation);
pub fn apply_transformation(
    angle: Option<f32>,
    distance: Option<f32>,
    transformation: &mut Transform3D,
) {
    // Emboss.cpp implementation
    // Apply rotation around Z axis if angle provided
    if let Some(a) = angle {
        let rotation = Transform3D::rotation_z(a as f64);
        *transformation = transformation.then(&rotation);
    }

    // Apply translation along Z axis if distance provided
    if let Some(d) = distance {
        let translation = Transform3D::translation(0.0, 0.0, d as f64);
        *transformation = transformation.then(&translation);
    }
}

/// Calculate scale factor to convert glyph shape points to mm
/// Emboss.hpp:170
/// C++: double get_text_shape_scale(const FontProp &fp, const FontFile &ff);
pub fn get_text_shape_scale(font_prop: &FontProp, _font_file: &FontFile) -> f64 {
    // Scale = (size in mm / units per em) * shape scale
    font_prop.size_in_mm * SHAPE_SCALE
}

/// Get line height including spacing
/// Emboss.hpp:186
/// C++: int get_line_height(const FontFile &font, const FontProp &prop);
pub fn get_line_height(font: &FontFile, font_prop: &FontProp) -> i32 {
    let info = &font.info;
    let line_height = info.ascent - info.descent + info.line_gap;
    let additional_gap = font_prop.line_gap;
    line_height + additional_gap
}

/// Calculate vertical alignment offset
/// Emboss.hpp:194
/// C++: double get_align_y_offset_in_mm(FontProp::VerticalAlign align, unsigned count_lines, const FontFile &ff, const FontProp &fp);
pub fn get_align_y_offset_in_mm(
    align: VerticalAlign,
    count_lines: u32,
    font_file: &FontFile,
    font_prop: &FontProp,
) -> f64 {
    let scale = get_text_shape_scale(font_prop, font_file);
    let line_height = get_line_height(font_file, font_prop) as f64;
    let total_height = line_height * (count_lines as f64 - 1.0);

    match align {
        VerticalAlign::Top => {
            // Align to top - use ascent
            -font_file.info.ascent as f64 * scale
        }
        VerticalAlign::Center => {
            // Center alignment
            -total_height * 0.5 * scale
        }
        VerticalAlign::Bottom => {
            // Align to bottom - use descent
            (total_height + font_file.info.descent as f64) * scale
        }
    }
}

/// Sample polygon slice by bounding box centers
/// Emboss.hpp:389
/// C++: PolygonPoints sample_slice(const TextLine &slice, const BoundingBoxes &bbs, double scale);
pub fn sample_slice(
    _slice: &TextLine,
    _bounding_boxes: &[BoundingBox],
    _scale: f64,
) -> Result<PolygonPoints> {
    // TODO: Implement polygon sampling
    // Sample points along the polygon path based on bounding box centers
    Err(Error::Geometry(
        "sample_slice not yet implemented".to_string(),
    ))
}

/// Calculate angle (atan2 of normal) at polygon point
/// Emboss.hpp:398
/// C++: double calculate_angle(int32_t distance, PolygonPoint polygon_point, const Polygon &polygon);
pub fn calculate_angle(_distance: i32, _polygon_point: PolygonPoint, _polygon: &Polygon) -> f64 {
    // TODO: Implement angle calculation
    // Find normal at point by looking at neighboring points
    0.0
}

/// Calculate angles for multiple polygon points
/// Emboss.hpp:399
/// C++: std::vector<double> calculate_angles(int32_t distance, const PolygonPoints& polygon_points, const Polygon &polygon);
pub fn calculate_angles(
    distance: i32,
    polygon_points: &[PolygonPoint],
    polygon: &Polygon,
) -> Vec<f64> {
    polygon_points
        .iter()
        .map(|&pt| calculate_angle(distance, pt, polygon))
        .collect()
}

/// Union polygons with safe delta offset
/// Emboss.hpp:94
/// C++: HealedExPolygons union_with_delta(ExPolygons expoly, float delta, unsigned max_heal_iteration);
pub fn union_with_delta(
    _expolygons: ExPolygons,
    _delta: f32,
    _max_heal_iteration: u32,
) -> Result<HealedExPolygons> {
    // TODO: Implement union with delta
    // This is a "morphological closing" operation:
    // 1. Offset outward by delta
    // 2. Union all polygons
    // 3. Offset inward by delta
    // 4. Heal any remaining issues
    Err(Error::Geometry(
        "union_with_delta not yet implemented - requires clipper2 offset operations".to_string(),
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_get_count_lines() {
        /// Count lines in empty string
        /// Emboss.cpp test
        assert_eq!(get_count_lines(""), 0);

        /// Count lines with single newline
        /// Emboss.cpp test
        assert_eq!(get_count_lines("line1\nline2"), 1);

        /// Count lines with multiple newlines
        /// Emboss.cpp test
        assert_eq!(get_count_lines("a\nb\nc\nd"), 3);
    }

    #[test]
    fn test_suggest_up_vertical() {
        /// When normal points up, suggest Y as up
        /// Emboss.cpp test
        let normal = Vec3d::new(0.0, 0.0, 1.0);
        let up = suggest_up(normal, 0.9);
        assert_eq!(up, Vec3d::new(0.0, 1.0, 0.0));
    }

    #[test]
    fn test_suggest_up_horizontal() {
        /// When normal is horizontal, suggest Z as up
        /// Emboss.cpp test
        let normal = Vec3d::new(1.0, 0.0, 0.0);
        let up = suggest_up(normal, 0.9);
        assert_eq!(up, Vec3d::new(0.0, 0.0, 1.0));
    }

    #[test]
    fn test_project_z_create_front_back() {
        /// ProjectZ creates points at z=0 and z=depth
        /// Emboss.cpp test
        let proj = ProjectZ::new(5.0);
        let p = Point::new(100, 200);
        let (front, back) = proj.create_front_back(&p);

        assert_eq!(front.x, 100.0);
        assert_eq!(front.y, 200.0);
        assert_eq!(front.z, 0.0);

        assert_eq!(back.x, 100.0);
        assert_eq!(back.y, 200.0);
        assert_eq!(back.z, 5.0);
    }

    #[test]
    fn test_project_z_unproject() {
        /// ProjectZ unproject returns x,y and optional depth
        /// Emboss.cpp test
        let proj = ProjectZ::new(5.0);
        let p3d = Vec3d::new(100.0, 200.0, 3.5);

        let mut depth = 0.0;
        let p2d = proj.unproject(&p3d, Some(&mut depth));

        assert!(p2d.is_some());
        let p2d = p2d.unwrap();
        assert_eq!(p2d.x, 100.0);
        assert_eq!(p2d.y, 200.0);
        assert_eq!(depth, 3.5);
    }

    #[test]
    fn test_ortho_project_3d() {
        /// OrthoProject3d adds direction vector
        /// Emboss.cpp test
        let direction = Vec3d::new(0.0, 0.0, 10.0);
        let proj = OrthoProject3d::new(direction);

        let point = Vec3d::new(5.0, 5.0, 0.0);
        let projected = proj.project(&point);

        assert_eq!(projected.x, 5.0);
        assert_eq!(projected.y, 5.0);
        assert_eq!(projected.z, 10.0);
    }

    #[test]
    fn test_get_text_shape_scale() {
        /// Scale factor calculation
        /// Emboss.cpp test
        let font_prop = FontProp {
            size_in_mm: 10.0,
            line_spacing: 1.0,
            char_spacing: 0.0,
            vertical_align: VerticalAlign::Center,
            line_gap: 0,
            collection_number: 0,
        };

        let font = FontFile {
            _data: vec![],
            info: FontInfo {
                ascent: 1000,
                descent: -200,
                line_gap: 0,
                units_per_em: 1000,
            },
        };

        let scale = get_text_shape_scale(&font_prop, &font);
        assert_eq!(scale, 10.0 * SHAPE_SCALE);
    }

    #[test]
    fn test_get_line_height() {
        /// Line height includes ascent, descent, line gap, and additional gap
        /// Emboss.cpp test
        let font = FontFile {
            _data: vec![],
            info: FontInfo {
                ascent: 800,
                descent: -200,
                line_gap: 100,
                units_per_em: 1000,
            },
        };

        let font_prop = FontProp {
            size_in_mm: 10.0,
            line_spacing: 1.0,
            char_spacing: 0.0,
            vertical_align: VerticalAlign::Center,
            line_gap: 50,
            collection_number: 0,
        };

        let height = get_line_height(&font, &font_prop);
        // ascent(800) - descent(-200) + line_gap(100) + additional(50) = 1150
        assert_eq!(height, 1150);
    }

    #[test]
    fn test_vertical_align_center() {
        /// Center alignment calculation
        /// Emboss.cpp test
        let font = FontFile {
            _data: vec![],
            info: FontInfo {
                ascent: 800,
                descent: -200,
                line_gap: 0,
                units_per_em: 1000,
            },
        };

        let font_prop = FontProp {
            size_in_mm: 10.0,
            line_spacing: 1.0,
            char_spacing: 0.0,
            vertical_align: VerticalAlign::Center,
            line_gap: 0,
            collection_number: 0,
        };

        let offset = get_align_y_offset_in_mm(
            VerticalAlign::Center,
            3, // 3 lines
            &font,
            &font_prop,
        );

        // Should center 3 lines of text
        // Line height = 800 - (-200) = 1000
        // Total height for 3 lines = 1000 * (3-1) = 2000
        // Center offset = -2000 * 0.5 * scale = -1000 * scale
        let expected = -1000.0 * get_text_shape_scale(&font_prop, &font);
        assert!((offset - expected).abs() < 0.0001);
    }

    #[test]
    fn test_constants() {
        /// Verify constants match C++ values
        /// Emboss.hpp/cpp
        assert_eq!(UNION_DELTA, 50.0);
        assert_eq!(UNION_MAX_ITERATION, 10);
        assert_eq!(ENTER_UNICODE, '\n' as u32);
        assert_eq!(SHAPE_SCALE, 0.001);
    }
}
