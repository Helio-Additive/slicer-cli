//! Tree support generation (2D algorithm).
//!
//! 1:1 faithful port of:
//! - Support/TreeSupport.hpp
//! - Support/TreeSupport.cpp
//!
//! This module implements the original 2D tree support algorithm. Tree supports
//! grow organic branch structures from overhang points down to the build plate,
//! using per-layer 2D collision avoidance.
//!
//! PORTING STATUS (partial): the standalone geometry helpers, the node and layer
//! height structures, `TreeSupportData` (the collision / avoidance caches and the
//! node arena), the profiler, and the `TreeSupport` members that work on the
//! object's layers and the node graph are ported in C++ order. What is left is
//! listed at the bottom of the file with what blocks it.
//!
//! The node graph. C++ links `SupportNode`s by raw pointers into
//! `TreeSupportData::contact_nodes` (`std::vector<std::unique_ptr<SupportNode>>`),
//! which owns every node. Here that vector owns the nodes by value and a
//! `SupportNode*` is the index of the node in it (`NodeId`); `nullptr` is `None`.
//! Nodes are never removed from the arena before `clear_nodes()`, so an index
//! stays valid exactly as long as the C++ pointer does.
//!
//! NOTE: no live code path calls into this file yet; the slicing pipeline still
//! runs `support/mod.rs` (`PrintObject::generate_support_material`).

use crate::clipper_utils::OffsetJoinType;
use crate::geometry::{
    get_extents_expoly, turn90_ccw, BoundingBox, ExPolygon, ExPolygons, Line, Lines, Point, Polygon,
    Polygons,
};
use crate::minimum_spanning_tree::MinimumSpanningTree;
use crate::{Coord, CoordF};
use crate::config::{ConfigOption, ConfigOptionFloat, ConfigOptionResolver};
use crate::layer::Layer;
use crate::print::faithful::{Print, PrintObject};
use crate::print_config::enums::{SupportMaterialPattern, SupportMaterialStyle, SupportType};
use crate::print_config::config_def::DynamicPrintConfig;
use crate::slicing::SlicingParams;
use crate::support::support_parameters::SupportParameters;
use crate::support_merge_plan::SupportMergePlan;
use std::cell::RefCell;
use std::collections::{BTreeMap, HashMap, HashSet};
use std::sync::Arc;

/// `SupportNode*`: the index of the node in `TreeSupportData::contact_nodes`.
pub type NodeId = usize;

// libslic3r.h:23 `static constexpr double SCALING_FACTOR = 0.000001;` as BambuStudio
// defines it (0.00001), not the crate's inverse `crate::SCALING_FACTOR`.
const SCALING_FACTOR: f64 = crate::libslic3r::SCALING_FACTOR;
// libslic3r.h:24 `static constexpr double EPSILON = 1e-4;`
const EPSILON: f64 = crate::libslic3r::EPSILON;
// ClipperLib::pftNonZero
const PFT_NON_ZERO: i32 = 1;

// ============================================================================
// ClipperUtils.hpp entry points used by this translation unit, over the
// vendored ClipperLib (the `*_clib` functions of clipper_utils). Deltas are the
// scaled `float` the C++ signature takes.
// ============================================================================

// ClipperUtils.hpp `ExPolygons union_ex(const ExPolygons &subject)`
fn union_ex(subject: &[ExPolygon]) -> ExPolygons {
    crate::clipper_utils::union_ex_clib(&to_polygons(subject), PFT_NON_ZERO)
}

// ClipperUtils.hpp `ExPolygons union_ex(const Polygons &subject)`
fn union_ex_polygons(subject: &[Polygon]) -> ExPolygons {
    crate::clipper_utils::union_ex_clib(subject, PFT_NON_ZERO)
}

// ClipperUtils.hpp `ExPolygons diff_ex(const ExPolygons &subject, const ExPolygons &clip)`
fn diff_ex(subject: &[ExPolygon], clip: &[ExPolygon]) -> ExPolygons {
    crate::clipper_utils::difference_clib(subject, clip)
}

// ClipperUtils.hpp `ExPolygons diff_ex(const ExPolygons &subject, const Polygons &clip)`
fn diff_ex_polygons(subject: &[ExPolygon], clip: &[Polygon]) -> ExPolygons {
    crate::clipper_utils::difference_clib(subject, &crate::clipper_utils::polygons_as_paths(clip))
}

// ClipperUtils.hpp `ExPolygons intersection_ex(const ExPolygons &subject, const ExPolygons &clip)`
fn intersection_ex(subject: &[ExPolygon], clip: &[ExPolygon]) -> ExPolygons {
    crate::clipper_utils::intersection_clib(subject, clip)
}

// ClipperUtils.hpp `ExPolygons intersection_ex(const ExPolygons &subject, const Polygons &clip)`
fn intersection_ex_polygons(subject: &[ExPolygon], clip: &[Polygon]) -> ExPolygons {
    crate::clipper_utils::intersection_clib(subject, &crate::clipper_utils::polygons_as_paths(clip))
}

// ClipperUtils.hpp `ExPolygons offset_ex(const ExPolygons &expolygons, const float delta,
// ClipperLib::JoinType joinType = DefaultJoinType, double miterLimit = DefaultMiterLimit)`
fn offset_ex(expolygons: &[ExPolygon], delta: f32) -> ExPolygons {
    offset_ex_jt(expolygons, delta, OffsetJoinType::Miter, 3.0)
}

fn offset_ex_jt(expolygons: &[ExPolygon], delta: f32, join_type: OffsetJoinType, miter_limit: f64) -> ExPolygons {
    crate::clipper_utils::offset_ex_expolygons_clib_ml(expolygons, delta as f64, join_type, miter_limit)
}

// ClipperUtils.hpp `Polygons offset(const ExPolygons &expolygons, const float delta, ...)`:
// each expolygon offset on its own, the paths concatenated.
fn offset_expolygons_jt(expolygons: &[ExPolygon], delta: f32, join_type: OffsetJoinType) -> Polygons {
    crate::clipper_utils::offset_expolygons_clib_raw_scaled(expolygons, delta as f64, join_type)
}

// ClipperUtils.hpp `Polygons offset(const Polygon &polygon, const float delta, ...)`
fn offset_polygon(polygon: &Polygon, delta: f32) -> Polygons {
    crate::clipper_utils::offset_polygons_clib(std::slice::from_ref(polygon), delta as f64, OffsetJoinType::Miter)
}

// ClipperUtils.hpp `ExPolygons offset2_ex(const ExPolygons &expolygons, const float delta1,
// const float delta2, ...)`. offset2_ex_clib takes the deltas in mm and truncates
// `mm / SCALING_FACTOR` to float, which gives back the scaled float handed in here.
fn offset2_ex(expolygons: &[ExPolygon], delta1: f32, delta2: f32) -> ExPolygons {
    crate::clipper_utils::offset2_ex_clib(
        expolygons,
        delta1 as f64 * SCALING_FACTOR,
        delta2 as f64 * SCALING_FACTOR,
        OffsetJoinType::Miter,
    )
}

// ClipperUtils.hpp `ExPolygons opening_ex(const ExPolygons &src, const float delta)`
// = offset2_ex(src, -delta, delta).
fn opening_ex(src: &[ExPolygon], delta: f32) -> ExPolygons {
    offset2_ex(src, -delta, delta)
}

// Polygon.hpp `Polygons to_polygons(const ExPolygons &src)`
fn to_polygons(src: &[ExPolygon]) -> Polygons {
    let mut out: Polygons = Vec::new();
    for expoly in src {
        out.push(expoly.contour.clone());
        for hole in &expoly.holes {
            out.push(hole.clone());
        }
    }
    out
}

// ExPolygon.hpp `void append(ExPolygons &dst, const ExPolygons &src)`
fn append(dst: &mut ExPolygons, src: ExPolygons) {
    dst.extend(src);
}

// ExPolygon.cpp:231-251 `Polygons ExPolygon::simplify_p(double tolerance) const`,
// tolerance scaled: Douglas-Peucker on the closed contour and holes, then
// `simplify_polygons(pp)` (ClipperLib::SimplifyPolygons, pftNonZero).
fn expolygon_simplify_p(expoly: &ExPolygon, tolerance: f64) -> Polygons {
    crate::clipper_utils::simplify_polygons_clib(&expoly.simplify_p_dp_rings_faithful(tolerance), PFT_NON_ZERO)
}

// ExPolygon.hpp:451-458 `ExPolygons expolygons_simplify(const ExPolygons &expolys, double tolerance)`:
// `exp.simplify(tolerance, &out)` = `append(out, union_ex(exp.simplify_p(tolerance)))`.
fn expolygons_simplify(expolys: &[ExPolygon], tolerance: f64) -> ExPolygons {
    let mut out: ExPolygons = Vec::with_capacity(expolys.len());
    for exp in expolys {
        append(&mut out, union_ex_polygons(&expolygon_simplify_p(exp, tolerance)));
    }
    out
}

// BoundingBox.hpp `double radius() const { return 0.5 * (max - min).cast<double>().norm(); }`
// Eigen's norm() sums the squares in its reduction functor, a separate expression
// from the products, so nothing is contracted.
fn bbox_radius(bbox: &BoundingBox) -> f64 {
    let dx = (bbox.max.x - bbox.min.x) as f64;
    let dy = (bbox.max.y - bbox.min.y) as f64;
    0.5 * (dx * dx + dy * dy).sqrt()
}

// Eigen `(v).cast<double>().norm()` of a Point.
fn point_norm(v: Point) -> f64 {
    let x = v.x as f64;
    let y = v.y as f64;
    (x * x + y * y).sqrt()
}

// ExPolygon.hpp: const Polygon& contour_or_hole(int i) const
// Returns the contour for i==0, otherwise holes[i-1]. (Mirrors BambuStudio's helper.)
#[inline]
fn contour_or_hole(expoly: &ExPolygon, i: usize) -> &Polygon {
    if i == 0 {
        &expoly.contour
    } else {
        &expoly.holes[i - 1]
    }
}

// TreeSupport.hpp:17-19 (SQ macro)
// #define SQ(x) ((x)*(x))
#[inline]
fn sq(x: f64) -> f64 {
    x * x
}

// TreeSupport.cpp:37-39
// #ifndef M_PI ... 3.1415926535897932384626433832795
const M_PI: f64 = std::f64::consts::PI;

// TreeSupport.cpp:40-42
// #define SIGN(x) (x>=0?1:-1)
#[inline]
fn sign(x: f64) -> i32 {
    if x >= 0.0 {
        1
    } else {
        -1
    }
}

// TreeSupport.cpp:43
// #define TAU (2.0 * M_PI)
const TAU: f64 = 2.0 * M_PI;

// TreeSupport.cpp:44
// #define NO_INDEX (std::numeric_limits<unsigned int>::max())
const NO_INDEX: u32 = u32::MAX;

// TreeSupport.cpp:45
// #define USE_SUPPORT_3D 0
const USE_SUPPORT_3D: bool = false;

// Branch radius limits (TreeSupport.hpp:447-450)
const MAX_BRANCH_RADIUS: CoordF = 10.0;
const MIN_BRANCH_RADIUS: CoordF = 0.4;
#[allow(dead_code)]
const MAX_BRANCH_RADIUS_FIRST_LAYER: CoordF = 12.0;
#[allow(dead_code)]
const MIN_BRANCH_RADIUS_FIRST_LAYER: CoordF = 2.0;

// TreeSupport.cpp:55
// #define unscale_(val) ((val) * SCALING_FACTOR)
#[inline]
fn unscale_(val: Coord) -> f64 {
    val as f64 * SCALING_FACTOR
}

// libslic3r.h:81 `#define scale_(val) ((val) / SCALING_FACTOR)`: a double; the
// call site converts it where the C++ does.
#[inline]
fn scale_(val: f64) -> f64 {
    val / SCALING_FACTOR
}

// Point.hpp:200 `Point operator*(const double &rhs) { return Point(this->x() * rhs, this->y() * rhs); }`,
// the member chosen for a non-const Point: Point(double, double) rounds with lrint.
#[inline]
fn point_mul(pt: Point, r: f64) -> Point {
    Point::new((pt.x as f64 * r).round_ties_even() as Coord, (pt.y as f64 * r).round_ties_even() as Coord)
}

// Point.hpp:179 `Point(double x, double y) : Vec2crd(coord_t(lrint(x)), coord_t(lrint(y)))`
#[inline]
fn point_from_f64(x: f64, y: f64) -> Point {
    Point::new(x.round_ties_even() as Coord, y.round_ties_even() as Coord)
}

// Point.cpp:157-180 `Point Point::projection_onto(const Line &line) const`. A local copy:
// `Point::project_onto_segment` of the crate takes the differences in 32 bits.
fn point_projection_onto_line(pt: &Point, a: Point, b: Point) -> Point {
    if a == b {
        return a;
    }
    let lx = (b.x - a.x) as f64;
    let ly = (b.y - a.y) as f64;
    // `( (double)(b(0) - x)*lx + (double)(b(1) - y)*ly ) / ( sqr<double>(lx) + sqr<double>(ly) )`:
    // the left product of the numerator is contracted with the sum, the squares are calls.
    let theta = ((b.x - pt.x) as f64).mul_add(lx, (b.y - pt.y) as f64 * ly) / (lx * lx + ly * ly);
    if 0.0 <= theta && theta <= 1.0 {
        // `(theta * a.cast<coordf_t>() + (1.0-theta) * b.cast<coordf_t>()).cast<coord_t>()`: Eigen's
        // functors, nothing contracted; the cast truncates.
        return Point::new(
            (theta * a.x as f64 + (1.0 - theta) * b.x as f64) as Coord,
            (theta * a.y as f64 + (1.0 - theta) * b.y as f64) as Coord,
        );
    }
    // Else pick closest endpoint.
    let sqn = |p: Point| {
        let (x, y) = ((p.x - pt.x) as f64, (p.y - pt.y) as f64);
        x * x + y * y
    };
    if sqn(a) < sqn(b) {
        a
    } else {
        b
    }
}

// Point.cpp:141-155 `Point Point::projection_onto(const MultiPoint &poly) const` on a Polygon:
// `poly.lines()` is virtual, Polygon::lines() (Polygon.hpp:199-209 `to_lines`) closes the contour and
// is empty under three points. A local copy: `Point::projection_onto_multipoint` of the crate leaves
// the closing edge out.
fn point_projection_onto_polygon(pt: &Point, poly: &Polygon) -> Point {
    let mut running_projection = poly.points[0];
    let mut running_min = point_norm(Point::new(running_projection.x - pt.x, running_projection.y - pt.y));
    let n = poly.points.len();
    if n > 2 {
        for i in 0..n {
            let (a, b) = (poly.points[i], poly.points[if i + 1 == n { 0 } else { i + 1 }]);
            let point_temp = point_projection_onto_line(pt, a, b);
            if point_norm(Point::new(point_temp.x - pt.x, point_temp.y - pt.y)) < running_min {
                running_projection = point_temp;
                running_min = point_norm(Point::new(running_projection.x - pt.x, running_projection.y - pt.y));
            }
        }
    }
    running_projection
}

// ExPolygon.cpp:206-223 `Point projection_onto(const ExPolygons& polygons, const Point& from)`
pub fn projection_onto(polygons: &[ExPolygon], from: &Point) -> Point {
    let mut projected_pt = Point::new(0, 0);
    let mut min_dist = f64::MAX;
    for poly in polygons {
        for i in 0..poly.num_contours() {
            let p = point_projection_onto_polygon(from, contour_or_hole(poly, i));
            let (dx, dy) = ((from.x - p.x) as f64, (from.y - p.y) as f64);
            let dist = dx * dx + dy * dy;
            if dist < min_dist {
                projected_pt = p;
                min_dist = dist;
            }
        }
    }
    projected_pt
}

// ============================================================================
// Header data structures (TreeSupport.hpp)
// ============================================================================

/// TreeSupport.hpp:27-37: struct LayerHeightData
#[derive(Debug, Clone, Default)]
pub struct LayerHeightData {
    // TreeSupport.hpp:29: coordf_t print_z = 0;
    pub print_z: CoordF,
    // TreeSupport.hpp:30: coordf_t height = 0;
    pub height: CoordF,
    // TreeSupport.hpp:31: size_t obj_layer_nr = 0;
    pub obj_layer_nr: usize,
}

impl LayerHeightData {
    // TreeSupport.hpp:32: LayerHeightData() = default;
    pub fn new() -> Self {
        Self::default()
    }

    // TreeSupport.hpp:33
    // LayerHeightData(coordf_t z, coordf_t h, size_t obj_layer) : print_z(z), height(h), obj_layer_nr(obj_layer) {}
    pub fn from_values(z: CoordF, h: CoordF, obj_layer: usize) -> Self {
        Self {
            print_z: z,
            height: h,
            obj_layer_nr: obj_layer,
        }
    }

    // TreeSupport.hpp:34-36
    // coordf_t bottom_z() { return print_z - height; }
    pub fn bottom_z(&self) -> CoordF {
        self.print_z - self.height
    }
}

/// TreeSupport.hpp:39-43: enum TreeNodeType
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TreeNodeType {
    // TreeSupport.hpp:40
    ECircle,
    // TreeSupport.hpp:41
    ESquare,
    // TreeSupport.hpp:42
    EPolygon,
}

impl Default for TreeNodeType {
    fn default() -> Self {
        // SupportNode default: type = eCircle (TreeSupport.hpp:125)
        Self::ECircle
    }
}

/// TreeSupport.cpp / TreeSupport.hpp:422: enum OverhangType : uint8_t
///
/// These are flag bits combined bitwise (Cantilever == 1<<1 etc.).
/// Note: Cantilever and SharpTail==1 share the bit pattern from the header;
/// reproduced verbatim.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OverhangType(pub u8);

impl OverhangType {
    // TreeSupport.hpp:422
    pub const NORMAL: OverhangType = OverhangType(0);
    pub const SHARP_TAIL: OverhangType = OverhangType(1);
    pub const CANTILEVER: OverhangType = OverhangType(1 << 1);
    pub const SMALL: OverhangType = OverhangType(1 << 2);
    pub const BIG_FLAT: OverhangType = OverhangType(1 << 3);
    pub const THIN_PLATE: OverhangType = OverhangType(1 << 4);
    pub const SHARP_TAIL_LOWESST: OverhangType = OverhangType(1 << 5);
}

impl Default for OverhangType {
    fn default() -> Self {
        Self::NORMAL
    }
}

/// TreeSupport.hpp:47-218: struct SupportNode
///
/// Represents the metadata of a node in the tree. The pointers of the C++ struct
/// (`parent`, `parents`, `child`, `merged_neighbours`) are `NodeId`s into the arena
/// `TreeSupportData::contact_nodes`, see the module documentation.
#[derive(Debug, Clone)]
pub struct SupportNode {
    /// TreeSupport.hpp:110-114
    /// The number of layers to go to the top of this branch.
    /// Negative value means it's a virtual node between support and overhang, which doesn't need to be extruded.
    pub distance_to_top: i32,
    // TreeSupport.hpp:115 coordf_t dist_mm_to_top = 0;  // dist to bottom contact in mm
    pub dist_mm_to_top: CoordF,

    // TreeSupport.hpp:123 Point position;
    pub position: Point,
    // TreeSupport.hpp:124 Point movement; // movement towards neighbor center or outline
    pub movement: Point,
    // TreeSupport.hpp:125 Point orig_pos;
    pub orig_pos: Point,
    // TreeSupport.hpp:126 mutable double radius = 0.0;
    pub radius: f64,
    // TreeSupport.hpp:127 mutable double max_move_dist = 0.0;
    pub max_move_dist: f64,
    // TreeSupport.hpp:128 TreeNodeType type = eCircle;
    pub type_: TreeNodeType,
    // TreeSupport.hpp:129 bool is_corner = false;
    pub is_corner: bool,
    // TreeSupport.hpp:130 bool is_processed = false;
    pub is_processed: bool,
    // TreeSupport.hpp:131 bool need_extra_wall = false;
    pub need_extra_wall: bool,
    // TreeSupport.hpp:132 bool is_sharp_tail = false;
    pub is_sharp_tail: bool,
    // TreeSupport.hpp:133 bool valid = true;
    pub valid: bool,
    // TreeSupport.hpp:134 float carried_mass = 0.0;  // Virtual carried mass (supported overhang area)
    // used to prioritize light-to-heavy branch merging
    pub carried_mass: f32,

    // TreeSupport.hpp:144-148 BBS multi-diameter: set by fold_nodes_multi_diameter(). slab_height > 0 marks
    // a node that carries a whole slab: its disk is one extrusion spanning [print_z - slab_height, print_z].
    pub slab_height: CoordF,
    // TreeSupport.hpp:149-150 Lowest object layer the slab covers, for the span collision lookup.
    pub slab_bottom_obj_layer: usize,
    // TreeSupport.hpp:151-153 Position of this node on the folded grid, counted from the bed.
    pub slab_index: i32,
    // TreeSupport.hpp:154-156 BBS multi-diameter: wall line width of this base node when it sits next to a
    // nozzle change, set by smooth_line_width_multi_diameter(). 0 keeps the layer's default support line width.
    pub line_width: CoordF,
    // TreeSupport.hpp:157 bool fading = false;
    pub fading: bool,
    // TreeSupport.hpp:158 double overhang_degree = 0.0;  // overhang degree for cooling just like perimeter
    pub overhang_degree: f64,
    // TreeSupport.hpp:159 ExPolygon overhang; // when type==ePolygon, set this value to get original overhang area
    pub overhang: ExPolygon,
    // TreeSupport.hpp:160 coordf_t origin_area;
    pub origin_area: CoordF,
    // TreeSupport.hpp:161 coordf_t target_radius = -1.;
    pub target_radius: CoordF,

    // TreeSupport.hpp:169 Point skin_direction;
    pub skin_direction: Point,

    // TreeSupport.hpp:179 int support_roof_layers_below;
    pub support_roof_layers_below: i32,
    // TreeSupport.hpp:180 int obj_layer_nr;
    pub obj_layer_nr: i32,

    // TreeSupport.hpp:189 bool to_buildplate;
    pub to_buildplate: bool,

    // TreeSupport.hpp:198 SupportNode* parent;
    pub parent: Option<NodeId>,
    // TreeSupport.hpp:199 std::vector<SupportNode*> parents;
    pub parents: Vec<NodeId>,
    // TreeSupport.hpp:200 SupportNode* child = nullptr;
    pub child: Option<NodeId>,

    // TreeSupport.hpp:209 std::list<SupportNode*> merged_neighbours;
    pub merged_neighbours: Vec<NodeId>,

    // TreeSupport.hpp:211 coordf_t print_z;
    pub print_z: CoordF,
    // TreeSupport.hpp:212 coordf_t height;
    pub height: CoordF,
}

// TreeSupport.hpp:118 `static double diameter_angle_scale_factor;` (defined TreeSupport.cpp:5101,
// set by drop_nodes() TreeSupport.cpp:5162). A process-wide static as in C++, kept as the bits of the double.
static SUPPORT_NODE_DIAMETER_ANGLE_SCALE_FACTOR: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

impl SupportNode {
    // TreeSupport.hpp:52 static constexpr SupportNode* NO_PARENT = nullptr;
    pub const NO_PARENT: Option<NodeId> = None;

    /// `SupportNode::diameter_angle_scale_factor`
    pub fn diameter_angle_scale_factor() -> f64 {
        f64::from_bits(SUPPORT_NODE_DIAMETER_ANGLE_SCALE_FACTOR.load(std::sync::atomic::Ordering::Relaxed))
    }

    /// `SupportNode::diameter_angle_scale_factor = value;`
    pub fn set_diameter_angle_scale_factor(value: f64) {
        SUPPORT_NODE_DIAMETER_ANGLE_SCALE_FACTOR.store(value.to_bits(), std::sync::atomic::Ordering::Relaxed);
    }

    // TreeSupport.hpp:54-64: SupportNode() default constructor
    pub fn new() -> Self {
        Self {
            distance_to_top: 0,
            dist_mm_to_top: 0.0,
            position: Point::new(0, 0),
            movement: Point::new(0, 0),
            orig_pos: Point::new(0, 0),
            radius: 0.0,
            max_move_dist: 0.0,
            type_: TreeNodeType::ECircle,
            is_corner: false,
            is_processed: false,
            need_extra_wall: false,
            is_sharp_tail: false,
            valid: true,
            carried_mass: 0.0,
            slab_height: 0.0,
            slab_bottom_obj_layer: 0,
            slab_index: -1,
            line_width: 0.0,
            fading: false,
            overhang_degree: 0.0,
            overhang: ExPolygon::default(),
            origin_area: 0.0,
            target_radius: -1.0,
            skin_direction: Point::new(0, 0),
            support_roof_layers_below: 0,
            obj_layer_nr: 0,
            to_buildplate: true,
            parent: None,
            parents: Vec::new(),
            child: None,
            merged_neighbours: Vec::new(),
            print_z: 0.0,
            height: 0.0,
        }
    }

    /// TreeSupport.hpp:66-100: the full SupportNode constructor.
    ///
    /// `nodes` is the arena the new node will be pushed onto at index `self_idx`
    /// (`this`); the constructor writes `parent->child` and the merged neighbours'
    /// `child` there, as the C++ does through the pointers.
    #[allow(clippy::too_many_arguments)]
    pub fn with_parent(
        nodes: &mut [SupportNode],
        self_idx: NodeId,
        position: Point,
        distance_to_top: i32,
        obj_layer_nr: i32,
        support_roof_layers_below: i32,
        to_buildplate: bool,
        parent: Option<NodeId>,
        print_z_: CoordF,
        height_: CoordF,
        dist_mm_to_top_: CoordF,
        radius_: CoordF,
    ) -> Self {
        let mut node = SupportNode {
            distance_to_top,
            position,
            orig_pos: position,
            obj_layer_nr,
            support_roof_layers_below,
            to_buildplate,
            parent,
            print_z: print_z_,
            height: height_,
            dist_mm_to_top: dist_mm_to_top_,
            radius: radius_,
            ..SupportNode::new()
        };

        // TreeSupport.hpp:81-99
        if let Some(parent_idx) = parent {
            // parents.push_back(parent);
            node.parents.push(parent_idx);
            // type = parent->type;
            node.type_ = nodes[parent_idx].type_;
            // overhang = parent->overhang;
            node.overhang = nodes[parent_idx].overhang.clone();
            // orig_pos = parent->orig_pos;
            node.orig_pos = nodes[parent_idx].orig_pos;
            // carried_mass = parent->carried_mass;
            node.carried_mass = nodes[parent_idx].carried_mass;
            // if (dist_mm_to_top == 0) dist_mm_to_top = parent->dist_mm_to_top + parent->height;
            if node.dist_mm_to_top == 0.0 {
                node.dist_mm_to_top = nodes[parent_idx].dist_mm_to_top + nodes[parent_idx].height;
            }
            // if (radius == 0 && parent->radius>0)
            //     radius = parent->radius + (dist_mm_to_top - parent->dist_mm_to_top) * diameter_angle_scale_factor;
            if node.radius == 0.0 && nodes[parent_idx].radius > 0.0 {
                node.radius = (node.dist_mm_to_top - nodes[parent_idx].dist_mm_to_top)
                    .mul_add(Self::diameter_angle_scale_factor(), nodes[parent_idx].radius);
            }
            // parent->child = this;
            nodes[parent_idx].child = Some(self_idx);
            // for (auto& neighbor : parent->merged_neighbours) {
            //     neighbor->child = this; parents.push_back(neighbor); carried_mass += neighbor->carried_mass; }
            let neighbours = nodes[parent_idx].merged_neighbours.clone();
            for neighbor in neighbours {
                nodes[neighbor].child = Some(self_idx);
                node.parents.push(neighbor);
                node.carried_mass += nodes[neighbor].carried_mass;
            }
            // is_sharp_tail = parent->is_sharp_tail;
            node.is_sharp_tail = nodes[parent_idx].is_sharp_tail;
            // skin_direction = parent->skin_direction;
            node.skin_direction = nodes[parent_idx].skin_direction;
        }

        node
    }

    /// TreeSupport.hpp:136-142 `float total_carried_mass() const`
    pub fn total_carried_mass(&self, nodes: &[SupportNode]) -> f32 {
        let mut mass = self.carried_mass;
        for &neighbor in &self.merged_neighbours {
            mass += nodes[neighbor].carried_mass;
        }
        mass
    }

    // TreeSupport.hpp:214-217 bool operator==(const SupportNode& other) const { return position == other.position; }
    pub fn eq_position(&self, other: &SupportNode) -> bool {
        self.position == other.position
    }
}

impl Default for SupportNode {
    fn default() -> Self {
        Self::new()
    }
}

impl PartialEq for SupportNode {
    // TreeSupport.hpp:214-217
    fn eq(&self, other: &Self) -> bool {
        self.position == other.position
    }
}

/// TreeSupport.hpp:355-360: struct LineHash
///
/// size_t operator()(const Line& line) const {
///     return (hash(line.a(0)) ^ hash(line.b(1))) * 102 +
///            (hash(line.a(1)) ^ hash(line.b(0))) * 10222;
/// }
#[derive(Debug, Clone, Default)]
pub struct LineHash;

impl LineHash {
    pub fn new() -> Self {
        Self
    }

    pub fn hash(&self, line: &Line) -> u64 {
        // std::hash<coord_t>() on int32_t coords (libslic3r.h:40) is the identity.
        // FIDELITY-NOTE(F2): coord_t is int32_t — reproduce the int32 truncation locally.
        let a0 = (line.a.x as i32) as u64;
        let a1 = (line.a.y as i32) as u64;
        let b0 = (line.b.x as i32) as u64;
        let b1 = (line.b.y as i32) as u64;
        (a0 ^ b1).wrapping_mul(102).wrapping_add((a1 ^ b0).wrapping_mul(10222))
    }
}

/// TreeSupport.hpp:267-272: struct RadiusLayerPair (key for the collision/avoidance caches)
#[derive(Debug, Clone, Copy, Default)]
pub struct RadiusLayerPair {
    // TreeSupport.hpp:268: coordf_t radius;
    pub radius: CoordF,
    // TreeSupport.hpp:269: size_t layer_nr;
    pub layer_nr: usize,
    // TreeSupport.hpp:270: int recursions;
    pub recursions: i32,
}

impl RadiusLayerPair {
    // TreeSupport.hpp:273-277: RadiusLayerPairEquality
    // bool operator()(_Left, _Right) const { return _Left.radius == _Right.radius && _Left.layer_nr == _Right.layer_nr; }
    pub fn equals(&self, other: &RadiusLayerPair) -> bool {
        self.radius == other.radius && self.layer_nr == other.layer_nr
    }

    // TreeSupport.hpp:278-282: RadiusLayerPairHash
    // size_t operator()(const RadiusLayerPair& elem) const {
    //     return std::hash<coord_t>()(elem.radius) ^ std::hash<coord_t>()(elem.layer_nr * 7919);
    // }
    pub fn hash(&self) -> u64 {
        // C++ casts radius (coordf_t/double) through std::hash<coord_t>(): radius is first
        // converted to coord_t (int32_t per libslic3r.h:40) then hashed (identity for ints).
        // FIDELITY-NOTE(F2): coord_t is int32_t — reproduce the int32 truncation locally.
        ((self.radius as i32) as u64) ^ ((self.layer_nr.wrapping_mul(7919)) as u64)
    }
}

impl RadiusLayerPair {
    /// The part of the key the cache compares (RadiusLayerPairEquality): the radius and the layer.
    fn cache_key(&self) -> (u64, usize) {
        (self.radius.to_bits(), self.layer_nr)
    }
}

/// TreeSupport.hpp:225-379 `class TreeSupportData`: lazily generates tree guidance volumes.
///
/// The caches are `mutable tbb::concurrent_unordered_map`s filled from const methods in C++;
/// here they are `RefCell`s, and a lookup hands back a copy of the cached areas.
#[derive(Debug)]
pub struct TreeSupportData {
    // TreeSupport.hpp:284 std::vector<LayerHeightData> layer_heights;
    pub layer_heights: Vec<LayerHeightData>,
    // TreeSupport.hpp:286 std::vector<std::unique_ptr<SupportNode>> contact_nodes;
    pub contact_nodes: Vec<SupportNode>,
    // TreeSupport.hpp:336 bool is_slim = false;
    pub is_slim: bool,
    // TreeSupport.hpp:340 coordf_t m_xy_distance;
    pub m_xy_distance: CoordF,
    // TreeSupport.hpp:342 double branch_scale_factor = 1.0; // tan(45 degrees)
    pub branch_scale_factor: f64,
    // TreeSupport.hpp:351 coordf_t m_radius_sample_resolution;
    pub m_radius_sample_resolution: CoordF,
    // TreeSupport.hpp:356 std::vector<ExPolygons> m_layer_outlines;
    pub m_layer_outlines: Vec<ExPolygons>,
    // TreeSupport.hpp:359 std::vector<ExPolygons> m_layer_outlines_below; // union contours of all layers below
    pub m_layer_outlines_below: Vec<ExPolygons>,
    // TreeSupport.hpp:361 std::vector<double> m_max_move_distances;
    pub m_max_move_distances: Vec<f64>,
    // TreeSupport.hpp:375-376 the collision and avoidance caches, keyed by (radius, layer_nr).
    pub m_collision_cache: RefCell<HashMap<(u64, usize), Arc<ExPolygons>>>,
    pub m_avoidance_cache: RefCell<HashMap<(u64, usize), Arc<ExPolygons>>>,
}

impl Default for TreeSupportData {
    // TreeSupport.hpp:228 TreeSupportData() = default;
    fn default() -> Self {
        TreeSupportData {
            layer_heights: Vec::new(),
            contact_nodes: Vec::new(),
            is_slim: false,
            m_xy_distance: 0.0,
            branch_scale_factor: 1.0,
            m_radius_sample_resolution: 0.0,
            m_layer_outlines: Vec::new(),
            m_layer_outlines_below: Vec::new(),
            m_max_move_distances: Vec::new(),
            m_collision_cache: RefCell::new(HashMap::new()),
            m_avoidance_cache: RefCell::new(HashMap::new()),
        }
    }
}

// TreeSupport.cpp:70-83 enum TreeSupportStage
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u32)]
pub enum TreeSupportStage {
    StageDetectOverhangs = 0,
    StageGenerateContactNodes,
    StageDropDownNodes,
    StageDrawCircles,
    StageGenerateToolpaths,
    StageMinimumSpanningTree,
    StageGetAvoidance,
    StageProjectionOntoEx,
    StageGetCollision,
    StageIntersectionLn,
    StageTotal,
    NumStages,
}

const NUM_STAGES: usize = TreeSupportStage::NumStages as usize;

/// TreeSupport.cpp:87-150 `class TreeSupportProfiler`. The boost local clock is
/// `std::time::Instant`; the durations are whole milliseconds as `total_milliseconds()`.
#[derive(Debug, Clone)]
pub struct TreeSupportProfiler {
    pub stage_durations: [u32; NUM_STAGES],
    pub stage_index: u32,
    pub tic_time: Option<std::time::Instant>,
    pub toc_time: Option<std::time::Instant>,
    m_stage_start_times: [Option<std::time::Instant>; NUM_STAGES],
}

impl TreeSupportProfiler {
    // TreeSupport.cpp:96-101
    pub const fn new() -> Self {
        TreeSupportProfiler {
            stage_durations: [0; NUM_STAGES],
            stage_index: 0,
            tic_time: None,
            toc_time: None,
            m_stage_start_times: [None; NUM_STAGES],
        }
    }

    // TreeSupport.cpp:103-109
    pub fn stage_start(&mut self, stage: TreeSupportStage) {
        if stage > TreeSupportStage::NumStages {
            return;
        }
        // TreeSupport.cpp:108 (NUM_STAGES itself passes the check and indexes one past the
        // array in C++; it is never passed)
        if let Some(slot) = self.m_stage_start_times.get_mut(stage as usize) {
            *slot = Some(std::time::Instant::now());
        }
    }

    // TreeSupport.cpp:111-118
    pub fn stage_finish(&mut self, stage: TreeSupportStage) {
        if stage > TreeSupportStage::NumStages {
            return;
        }
        let time = std::time::Instant::now();
        let idx = stage as usize;
        if idx < NUM_STAGES {
            let start = self.m_stage_start_times[idx].unwrap_or(time);
            self.stage_durations[idx] = time.duration_since(start).as_millis() as u32;
        }
    }

    // TreeSupport.cpp:120
    pub fn tic(&mut self) {
        self.tic_time = Some(std::time::Instant::now());
    }

    // TreeSupport.cpp:121-124
    pub fn toc(&mut self) -> u32 {
        let now = std::time::Instant::now();
        self.toc_time = Some(now);
        now.duration_since(self.tic_time.unwrap_or(now)).as_millis() as u32
    }

    // TreeSupport.cpp:125-130
    pub fn stage_add(&mut self, stage: TreeSupportStage) {
        if stage > TreeSupportStage::NumStages {
            return;
        }
        let elapsed = self.toc();
        if let Some(d) = self.stage_durations.get_mut(stage as usize) {
            *d = d.wrapping_add(elapsed);
        }
    }

    // TreeSupport.cpp:132-147
    pub fn report(&self) -> String {
        format!(
            "total overhange cost: {}; STAGE_DETECT_OVERHANGS: {}; STAGE_GENERATE_CONTACT_NODES: {}; STAGE_DROP_DOWN_NODES: {}; STAGE_DRAW_CIRCLES: {}; STAGE_GENERATE_TOOLPATHS: {}; STAGE_MinimumSpanningTree: {}; STAGE_GET_AVOIDANCE: {}; STAGE_projection_onto_ex: {}; STAGE_get_collision: {}; STAGE_intersection_ln: {}",
            self.stage_durations[TreeSupportStage::StageTotal as usize],
            self.stage_durations[TreeSupportStage::StageDetectOverhangs as usize],
            self.stage_durations[TreeSupportStage::StageGenerateContactNodes as usize],
            self.stage_durations[TreeSupportStage::StageDropDownNodes as usize],
            self.stage_durations[TreeSupportStage::StageDrawCircles as usize],
            self.stage_durations[TreeSupportStage::StageGenerateToolpaths as usize],
            self.stage_durations[TreeSupportStage::StageMinimumSpanningTree as usize],
            self.stage_durations[TreeSupportStage::StageGetAvoidance as usize],
            self.stage_durations[TreeSupportStage::StageProjectionOntoEx as usize],
            self.stage_durations[TreeSupportStage::StageGetCollision as usize],
            self.stage_durations[TreeSupportStage::StageIntersectionLn as usize],
        )
    }
}

impl Default for TreeSupportProfiler {
    fn default() -> Self {
        Self::new()
    }
}

// TreeSupport.cpp:152 `TreeSupportProfiler profiler;`
static PROFILER: std::sync::Mutex<TreeSupportProfiler> = std::sync::Mutex::new(TreeSupportProfiler::new());

fn profiler() -> std::sync::MutexGuard<'static, TreeSupportProfiler> {
    PROFILER.lock().unwrap_or_else(|e| e.into_inner())
}

// ============================================================================
// TreeSupport.cpp free functions / static helpers
// ============================================================================

// TreeSupport.cpp:59-62
// inline double dot_with_unscale(const Point a, const Point b)
// { return unscale_(a(0)) * unscale_(b(0)) + unscale_(a(1)) * unscale_(b(1)); }
#[inline]
pub fn dot_with_unscale(a: Point, b: Point) -> f64 {
    // The left product is contracted with the sum (clang -ffp-contract=on).
    unscale_(a.x).mul_add(unscale_(b.x), unscale_(a.y) * unscale_(b.y))
}

// TreeSupport.cpp:64-67
// inline double vsize2_with_unscale(const Point pt) { return dot_with_unscale(pt, pt); }
#[inline]
pub fn vsize2_with_unscale(pt: Point) -> f64 {
    dot_with_unscale(pt, pt)
}

// TreeSupport.cpp:69-74
// inline Point normal(Point pt, double scale)
// { double length = scale_(sqrt(vsize2_with_unscale(pt))); return pt * (scale / length); }
#[inline]
pub fn normal(pt: Point, scale_factor: f64) -> Point {
    // double length = scale_(sqrt(vsize2_with_unscale(pt)));
    let length = scale_(vsize2_with_unscale(pt).sqrt());
    // return pt * (scale / length);  `pt` is a non-const Point: the member operator* (lrint).
    point_mul(pt, scale_factor / length)
}

// TreeSupport.cpp:157-183
// Lines spanning_tree_to_lines(const std::vector<MinimumSpanningTree>& spanning_trees)
pub fn spanning_tree_to_lines(spanning_trees: &[MinimumSpanningTree]) -> Lines {
    // Lines polylines;
    let mut polylines: Lines = Vec::new();
    // for (const MinimumSpanningTree& mst : spanning_trees) {
    for mst in spanning_trees {
        // std::vector<Point> points = mst.vertices();
        let points = mst.vertices();
        // std::unordered_set<Point, PointHash> to_ignore;
        let mut to_ignore: HashSet<Point> = HashSet::new();
        // for (Point pt1 : points) {
        for pt1 in &points {
            // if (to_ignore.find(pt1) != to_ignore.end()) continue;
            if to_ignore.contains(pt1) {
                continue;
            }

            // const std::vector<Point>& neighbours = mst.adjacent_nodes(pt1);
            let neighbours = mst.adjacent_nodes(*pt1);
            // if (neighbours.empty()) continue;
            if neighbours.is_empty() {
                continue;
            }

            // for (Point pt2 : neighbours) {
            for pt2 in &neighbours {
                // if (to_ignore.find(pt2) != to_ignore.end()) continue;
                if to_ignore.contains(pt2) {
                    continue;
                }

                // Line line(pt1, pt2);
                // polylines.push_back(line);
                polylines.push(Line::new(*pt1, *pt2));
            }

            // to_ignore.insert(pt1);
            to_ignore.insert(*pt1);
        }
    }
    // return polylines;
    polylines
}

// TreeSupport.cpp:274-399
// Move point from inside polygon if distance>0, outside if distance<0.
// Special case: distance=0 means find the nearest point of from on the polygon contour.
// The max move distance should not excceed max_move_distance.
// @return success(true) or not(false)
pub fn move_inside_expoly(
    polygon: &ExPolygon,
    from: &mut Point,
    distance: f64,
    max_move_distance: f64,
) -> bool {
    //TODO: This is copied from the moveInside of Polygons.
    // Point ret = from;
    let mut ret = *from;
    // double bestDist2 = std::numeric_limits<double>::max();
    let mut best_dist2 = f64::MAX;
    // bool is_already_on_correct_side_of_boundary = false;
    let mut is_already_on_correct_side_of_boundary = false;
    // const Polygon &contour = polygon.contour;
    let contour = &polygon.contour;

    // if (contour.points.size() < 2) return false;
    if contour.points.len() < 2 {
        return false;
    }
    // Point p0 = contour.points[polygon.contour.size() - 2];
    let mut p0 = contour.points[polygon.contour.points.len() - 2];
    // Point p1 = contour.points.back();
    let mut p1 = *contour.points.last().unwrap();
    // bool projected_p_beyond_prev_segment = dot_with_unscale(p1 - p0, from - p0) >= vsize2_with_unscale(p1 - p0);
    let mut projected_p_beyond_prev_segment =
        dot_with_unscale(p1 - p0, *from - p0) >= vsize2_with_unscale(p1 - p0);
    // for(const Point& p2 : polygon.contour.points)
    for &p2 in &polygon.contour.points {
        // X = P projected on AB
        // const Point& a = p1; const Point& b = p2; const Point& p = from;
        let a = p1;
        let b = p2;
        let p = *from;
        // Point ab = b - a; Point ap = p - a;
        let ab = b - a;
        let ap = p - a;
        // double ab_length2 = vsize2_with_unscale(ab);
        let ab_length2 = vsize2_with_unscale(ab);
        // if(ab_length2 <= 0) { p1 = p2; continue; }
        if ab_length2 <= 0.0 {
            p1 = p2;
            continue;
        }
        // double dot_prod = dot_with_unscale(ab, ap);
        let dot_prod = dot_with_unscale(ab, ap);
        if dot_prod <= 0.0 {
            // x is projected to before ab
            if projected_p_beyond_prev_segment {
                //  case which looks like:   > .
                projected_p_beyond_prev_segment = false;
                // Point& x = p1;
                let x = p1;

                // double dist2 = vsize2_with_unscale(x - p);
                let dist2 = vsize2_with_unscale(x - p);
                if dist2 < best_dist2 {
                    best_dist2 = dist2;
                    if distance == 0.0 {
                        ret = x;
                    } else {
                        // Point inward_dir = turn90_ccw(normal(ab, 10.0) + normal(p1 - p0, 10.0));
                        let inward_dir = turn90_ccw(normal(ab, 10.0) + normal(p1 - p0, 10.0));
                        // ret = x + normal(inward_dir, scale_(distance));
                        ret = x + normal(inward_dir, scale_(distance) as f64);
                        // is_already_on_correct_side_of_boundary = dot_with_unscale(inward_dir, p - x) * distance >= 0;
                        is_already_on_correct_side_of_boundary =
                            dot_with_unscale(inward_dir, p - x) * distance >= 0.0;
                    }
                }
            } else {
                projected_p_beyond_prev_segment = false;
                p0 = p1;
                p1 = p2;
                continue;
            }
        } else if dot_prod >= ab_length2 {
            // x is projected to beyond ab
            projected_p_beyond_prev_segment = true;
            p0 = p1;
            p1 = p2;
            continue;
        } else {
            // x is projected to a point properly on the line segment. The case which looks like | .
            projected_p_beyond_prev_segment = false;
            // Point x = a + ab * (dot_prod / ab_length2);
            let x = a + point_mul(ab, dot_prod / ab_length2);

            // double dist2 = vsize2_with_unscale(p - x);
            let dist2 = vsize2_with_unscale(p - x);
            if dist2 < best_dist2 {
                best_dist2 = dist2;
                if distance == 0.0 {
                    ret = x;
                } else {
                    // Point inward_dir = turn90_ccw(normal(ab, scale_(distance)));
                    let inward_dir = turn90_ccw(normal(ab, scale_(distance) as f64));
                    // ret = x + inward_dir;
                    ret = x + inward_dir;
                    // is_already_on_correct_side_of_boundary = dot_with_unscale(inward_dir, p - x) >= 0;
                    is_already_on_correct_side_of_boundary =
                        dot_with_unscale(inward_dir, p - x) >= 0.0;
                }
            }
        }

        p0 = p1;
        p1 = p2;
    }

    if is_already_on_correct_side_of_boundary {
        // BBS. Remove this condition.
        if best_dist2 < distance * distance {
            *from = ret;
        }
        true
    } else if best_dist2 < max_move_distance * max_move_distance {
        *from = ret;
        true
    } else {
        false
    }
}

// TreeSupport.cpp:401-527
// Implementation assumes moving inside, but moving outside should just as well be possible.
pub fn move_inside_expolys(
    polygons: &ExPolygons,
    from: &mut Point,
    distance: f64,
    max_move_distance: f64,
) -> bool {
    // Point from0 = from;
    let from0 = *from;
    // Point ret = from;
    let mut ret = *from;
    // std::vector<Point> valid_pts;
    let mut valid_pts: Vec<Point> = Vec::new();
    // double bestDist2 = std::numeric_limits<double>::max();
    let mut best_dist2 = f64::MAX;
    // unsigned int bestPoly = NO_INDEX;
    let mut best_poly: u32 = NO_INDEX;
    // bool is_already_on_correct_side_of_boundary = false;
    let mut is_already_on_correct_side_of_boundary = false;
    // Point inward_dir;
    let mut inward_dir;
    // for (unsigned int poly_idx = 0; poly_idx < polygons.size(); poly_idx++)
    for poly_idx in 0..polygons.len() {
        // const ExPolygon poly = polygons[poly_idx];
        let poly = &polygons[poly_idx];
        // if (poly.contour.size() < 2) continue;
        if poly.contour.points.len() < 2 {
            continue;
        }
        // Point p0 = poly.contour[poly.contour.size()-2];
        let mut p0 = poly.contour.points[poly.contour.points.len() - 2];
        // Point p1 = poly.contour.points.back();
        let mut p1 = *poly.contour.points.last().unwrap();
        // bool projected_p_beyond_prev_segment = dot_with_unscale(p1 - p0, from - p0) >= vsize2_with_unscale(p1 - p0);
        let mut projected_p_beyond_prev_segment =
            dot_with_unscale(p1 - p0, *from - p0) >= vsize2_with_unscale(p1 - p0);
        // for(const Point p2 : poly.contour.points)
        for &p2 in &poly.contour.points {
            // Point a = p1; Point b = p2; Point p = from;
            let a = p1;
            let b = p2;
            let p = *from;
            // Point ab = b - a; Point ap = p - a;
            let ab = b - a;
            let ap = p - a;
            // double ab_length2 = vsize2_with_unscale(ab);
            let ab_length2 = vsize2_with_unscale(ab);
            // if(ab_length2 <= 0) { p1 = p2; continue; }
            if ab_length2 <= 0.0 {
                p1 = p2;
                continue;
            }
            // double dot_prod = dot_with_unscale(ab, ap);
            let dot_prod = dot_with_unscale(ab, ap);
            if dot_prod <= 0.0 {
                // x is projected to before ab
                if projected_p_beyond_prev_segment {
                    //  case which looks like:   > .
                    projected_p_beyond_prev_segment = false;
                    // Point& x = p1;
                    let x = p1;

                    // double dist2 = vsize2_with_unscale(x - p);
                    let dist2 = vsize2_with_unscale(x - p);
                    if dist2 < best_dist2 {
                        best_dist2 = dist2;
                        best_poly = poly_idx as u32;
                        if distance == 0.0 {
                            ret = x;
                        } else {
                            // inward_dir = turn90_ccw(normal(ab, 10.0) + normal(p1 - p0, 10.0));
                            inward_dir = turn90_ccw(normal(ab, 10.0) + normal(p1 - p0, 10.0));
                            // ret = x + normal(inward_dir, scale_(distance));
                            ret = x + normal(inward_dir, scale_(distance) as f64);
                            // is_already_on_correct_side_of_boundary = dot_with_unscale(inward_dir, p - x) * distance >= 0;
                            is_already_on_correct_side_of_boundary =
                                dot_with_unscale(inward_dir, p - x) * distance >= 0.0;
                            // if (is_already_on_correct_side_of_boundary && dist2 < distance * distance)
                            //     valid_pts.push_back(ret-from0);
                            if is_already_on_correct_side_of_boundary && dist2 < distance * distance {
                                valid_pts.push(ret - from0);
                            }
                        }
                    }
                } else {
                    projected_p_beyond_prev_segment = false;
                    p0 = p1;
                    p1 = p2;
                    continue;
                }
            } else if dot_prod >= ab_length2 {
                // x is projected to beyond ab
                projected_p_beyond_prev_segment = true;
                p0 = p1;
                p1 = p2;
                continue;
            } else {
                // x is projected to a point properly on the line segment. The case which looks like | .
                projected_p_beyond_prev_segment = false;
                // Point x = a + ab * (dot_prod / ab_length2);
                let x = a + point_mul(ab, dot_prod / ab_length2);

                // double dist2 = vsize2_with_unscale(p - x);
                let dist2 = vsize2_with_unscale(p - x);
                if dist2 < best_dist2 {
                    best_dist2 = dist2;
                    best_poly = poly_idx as u32;
                    if distance == 0.0 {
                        ret = x;
                    } else {
                        // inward_dir = turn90_ccw(normal(ab, scale_(distance)));
                        inward_dir = turn90_ccw(normal(ab, scale_(distance) as f64));
                        // ret = x + inward_dir;
                        ret = x + inward_dir;
                        // is_already_on_correct_side_of_boundary = dot_with_unscale(inward_dir, p - x) >= 0;
                        is_already_on_correct_side_of_boundary =
                            dot_with_unscale(inward_dir, p - x) >= 0.0;
                        // if (is_already_on_correct_side_of_boundary && dist2<distance*distance)
                        //     valid_pts.push_back(ret-from0);
                        if is_already_on_correct_side_of_boundary && dist2 < distance * distance {
                            valid_pts.push(ret - from0);
                        }
                    }
                }
            }
            p0 = p1;
            p1 = p2;
        }
    }

    // (commented-out valid_pts combine block left out, matching C++ #if 0)
    let _ = (&valid_pts, best_poly);

    if is_already_on_correct_side_of_boundary {
        if best_dist2 < distance * distance {
            *from = ret;
        }
        true
    } else if best_dist2 < max_move_distance * max_move_distance {
        *from = ret;
        true
    } else {
        false
    }
}

// MultiPoint.hpp:53-68: `closest_point_index` / `closest_point` return the closest
// *vertex* of the contour (by Euclidean norm), NOT the closest point projected onto an
// edge. The crate's `Polygon::closest_point` does edge projection, so we reproduce the
// vertex semantics locally for fidelity with C++ `MultiPoint::closest_point`.
fn polygon_closest_vertex(poly: &Polygon, point: &Point) -> Option<Point> {
    if poly.points.is_empty() {
        return None;
    }
    let mut idx = 0usize;
    // double dist_min = (point - points.front()).cast<double>().norm();
    let mut dist_min = point.distance(&poly.points[0]);
    for i in 1..poly.points.len() {
        // double d = (points[i] - point).cast<double>().norm();
        let d = point.distance(&poly.points[i]);
        if d < dist_min {
            dist_min = d;
            idx = i;
        }
    }
    Some(poly.points[idx])
}

// TreeSupport.cpp:529-546
// static Point find_closest_ex(Point from, const ExPolygons& polygons)
pub fn find_closest_ex(from: Point, polygons: &ExPolygons) -> Point {
    // Point closest_pt;
    let mut closest_pt = Point::new(0, 0);
    // double min_dist2 = std::numeric_limits<double>::max();
    let mut min_dist2 = f64::MAX;

    // for (const ExPolygon &poly : polygons) {
    for poly in polygons {
        // for (int i = 0; i < poly.num_contours(); i++) {
        for i in 0..poly.num_contours() {
            // const Point* candidate = poly.contour_or_hole(i).closest_point(from);
            let candidate = match polygon_closest_vertex(contour_or_hole(poly, i), &from) {
                Some(c) => c,
                None => continue,
            };
            // double dist2 = vsize2_with_unscale(*candidate - from);
            let dist2 = vsize2_with_unscale(candidate - from);
            if dist2 < min_dist2 {
                closest_pt = candidate;
                min_dist2 = dist2;
            }
        }
    }

    closest_pt
}

// TreeSupport.cpp:548-551
// static bool move_outside_expolys(const ExPolygons& polygons, Point& from, double distance, double max_move_distance)
// { return move_inside_expolys(polygons, from, -distance, -max_move_distance); }
pub fn move_outside_expolys(
    polygons: &ExPolygons,
    from: &mut Point,
    distance: f64,
    max_move_distance: f64,
) -> bool {
    move_inside_expolys(polygons, from, -distance, -max_move_distance)
}

// TreeSupport.cpp:553-559
// static bool is_inside_ex(const ExPolygon &polygon, const Point &pt)
pub fn is_inside_ex_poly(polygon: &ExPolygon, pt: &Point) -> bool {
    // if (!get_extents(polygon).contains(pt)) return false;
    if !get_extents_expoly(polygon).contains_point(pt) {
        return false;
    }
    // return polygon.contains(pt);  ExPolygon.cpp:109-119, border_result = true: a point on the
    // boundary of a hole is inside.
    polygon.contains(pt, true)
}

// TreeSupport.cpp:561-569
// static bool is_inside_ex(const ExPolygons &polygons, const Point &pt)
pub fn is_inside_ex(polygons: &ExPolygons, pt: &Point) -> bool {
    // for (const ExPolygon &poly : polygons) { if (is_inside_ex(poly, pt)) return true; }
    for poly in polygons {
        if is_inside_ex_poly(poly, pt) {
            return true;
        }
    }
    false
}

// TreeSupport.cpp:571-600
// use project_onto which is more accurate but more expensive
// static bool move_out_expolys(const ExPolygons& polygons, Point& from, double distance, double max_move_distance)
pub fn move_out_expolys(
    polygons: &ExPolygons,
    from: &mut Point,
    distance: f64,
    max_move_distance: f64,
) -> bool {
    // Point from0 = from;
    let _from0 = *from;
    // ExPolygons polys_dilated = union_ex(offset_ex(polygons, scale_(distance)));
    let polys_dilated = union_ex(&offset_ex(polygons, scale_(distance) as f32));
    // Point pt = projection_onto(polys_dilated, from);
    let mut pt = projection_onto(&polys_dilated, from);
    // Point outward_dir = pt - from;
    let outward_dir = pt - *from;
    // Point pt_max = from + normal(outward_dir, scale_(max_move_distance));
    let pt_max = *from + normal(outward_dir, scale_(max_move_distance) as f64);
    // double dist2 = vsize2_with_unscale(outward_dir);
    let dist2 = vsize2_with_unscale(outward_dir);
    // if (dist2 > SQ(max_move_distance)) pt = pt_max;
    if dist2 > sq(max_move_distance) {
        pt = pt_max;
    }
    // case 5: already outside and far enough, no need to move
    // if (!is_inside_ex(polys_dilated, from)) return true;
    if !is_inside_ex(&polys_dilated, from) {
        true
    }
    // else if (!is_inside_ex(polygons, from)) { from = pt; return true; }
    else if !is_inside_ex(polygons, from) {
        // case 4: already outside but not far enough
        *from = pt;
        true
    } else {
        // bool pt_max_in_poly = is_inside_ex(polygons, pt_max);
        let pt_max_in_poly = is_inside_ex(polygons, &pt_max);
        if !pt_max_in_poly {
            *from = pt_max;
            true
        } else {
            false
        }
    }
}

// TreeSupport.cpp:602-605
// static Point bounding_box_middle(const BoundingBox &bbox) { return (bbox.max + bbox.min) / 2; }
pub fn bounding_box_middle(bbox: &BoundingBox) -> Point {
    (bbox.max + bbox.min) / 2
}

// TreeSupport.cpp:650-658
// bool is_stable(float height, const ExPolygon &overhang, float strength_z)
pub fn is_stable(height: f32, overhang: &ExPolygon, _strength_z: f32) -> bool {
    // float stability = 1. * height;  // computed in double, stored as float
    let stability: f32 = (1.0_f64 * height as f64) as f32;
    // double Ixx = 0, Iyy = 0;
    let mut ixx = 0.0_f64;
    let mut iyy = 0.0_f64;
    // auto props = compSecondMoment({overhang}, Ixx, Iyy);  // Brim.cpp
    let _props =
        crate::brim::comp_second_moment_expolygons(&vec![overhang.clone()], &mut ixx, &mut iyy);
    // double moment = std::min(Ixx * pow(SCALING_FACTOR, 4), Iyy * pow(SCALING_FACTOR, 4));
    let moment = (ixx * SCALING_FACTOR.powf(4.0)).min(iyy * SCALING_FACTOR.powf(4.0));
    // return moment / stability > 3.;  (stability promoted float->double for the division)
    moment / stability as f64 > 3.0
}

// TreeSupport.cpp:2081-2099
// coordf_t TreeSupport::calc_branch_radius(base_radius, layers_to_top, tip_layers, diameter_angle_scale_factor)
//
// Ported as a free function. `is_slim` is the TreeSupport member flag (TreeSupport.hpp:457).
pub fn calc_branch_radius_by_layers(
    base_radius: CoordF,
    layers_to_top: usize,
    tip_layers: usize,
    diameter_angle_scale_factor: f64,
    is_slim: bool,
) -> CoordF {
    // double radius;
    let radius;
    if !is_slim {
        if (layers_to_top + 1) > tip_layers {
            // radius = base_radius + base_radius * (layers_to_top + 1) * diameter_angle_scale_factor;
            radius = base_radius
                + base_radius * (layers_to_top + 1) as f64 * diameter_angle_scale_factor;
        } else {
            // radius = base_radius * (layers_to_top + 1) / tip_layers;
            radius = base_radius * (layers_to_top + 1) as f64 / tip_layers as f64;
        }
    } else if (layers_to_top + 1) > tip_layers * 2 {
        // radius = base_radius + base_radius * (layers_to_top + 1) * diameter_angle_scale_factor;
        radius =
            base_radius + base_radius * (layers_to_top + 1) as f64 * diameter_angle_scale_factor;
    } else {
        // radius = base_radius * (layers_to_top + 1) / (tip_layers * 2);
        radius = base_radius * (layers_to_top + 1) as f64 / (tip_layers * 2) as f64;
    }
    // radius = std::clamp(radius, MIN_BRANCH_RADIUS, MAX_BRANCH_RADIUS);
    radius.clamp(MIN_BRANCH_RADIUS, MAX_BRANCH_RADIUS)
}

// TreeSupport.cpp:2101-2122
// coordf_t TreeSupport::calc_branch_radius(base_radius, mm_to_top, diameter_angle_scale_factor, use_min_distance)
//
// Ported as a free function. `support_interface_top_layers` is the object config
// value (TreeSupport.cpp:2115). The `m_model_volumes->ceilRadius` block is guarded
// by `#if USE_SUPPORT_3D` which is 0, so it is not executed.
pub fn calc_branch_radius_by_mm(
    base_radius: CoordF,
    mm_to_top: CoordF,
    diameter_angle_scale_factor: f64,
    _use_min_distance: bool,
    support_interface_top_layers: i32,
) -> CoordF {
    // double radius;
    let mut radius;
    // coordf_t tip_height = base_radius; // this is a 45 degree tip
    let tip_height = base_radius;
    if mm_to_top > tip_height {
        // radius = base_radius + (mm_to_top-tip_height) * diameter_angle_scale_factor;
        radius = base_radius + (mm_to_top - tip_height) * diameter_angle_scale_factor;
    } else {
        // radius = mm_to_top; // this is a 45 degree tip
        radius = mm_to_top;
    }
    // radius = std::clamp(radius, MIN_BRANCH_RADIUS, MAX_BRANCH_RADIUS);
    radius = radius.clamp(MIN_BRANCH_RADIUS, MAX_BRANCH_RADIUS);
    // if (m_object_config->support_interface_top_layers.value > 0) radius = std::max(radius, base_radius);
    if support_interface_top_layers > 0 {
        radius = radius.max(base_radius);
    }
    // #if USE_SUPPORT_3D ... ceilRadius ... #endif  (USE_SUPPORT_3D == 0, skipped)
    radius
}

// TreeSupport.cpp:2189-2206
// ExPolygons avoid_object_remove_extra_small_parts(const ExPolygon &expoly, const ExPolygons& avoid_region)
pub fn avoid_object_remove_extra_small_parts(
    expoly: &ExPolygon,
    avoid_region: &ExPolygons,
) -> ExPolygons {
    // ExPolygons expolys_out;
    let mut expolys_out: ExPolygons = Vec::new();
    // if(expoly.empty()) return expolys_out;
    if expoly.contour.points.is_empty() {
        return expolys_out;
    }
    // auto clipped_avoid_region = ClipperUtils::clip_clipper_polygons_with_subject_bbox(avoid_region, get_extents(expoly));
    // ClipperUtils.cpp:161-172: the ExPolygons overload flattens each ExPolygon's
    // contour+holes into a single Polygons list (mixed CW/CCW windings), which the
    // subsequent diff_ex interprets via Clipper's fill rule. We must reproduce that
    // flat Polygons clip, not wrap each polygon as an independent positive contour.
    let subject_bbox = get_extents_expoly(expoly);
    let clipped_avoid_region: Vec<Polygon> =
        crate::clipper_utils::clip_clipper_polygons_with_subject_bbox_expolygons(
            avoid_region,
            &subject_bbox,
            false,
        );
    // auto expolys_avoid = diff_ex(expoly, clipped_avoid_region);
    let expolys_avoid = diff_ex_polygons(std::slice::from_ref(expoly), &clipped_avoid_region);
    // int idx_max_area = -1;
    let mut idx_max_area: i32 = -1;
    // float max_area = 0;
    let mut max_area: f32 = 0.0;
    // for (int i = 0; i < expolys_avoid.size(); ++i)
    for (i, ea) in expolys_avoid.iter().enumerate() {
        // auto a = expolys_avoid[i].area();  // ExPolygon::area() returns double
        let a: f64 = ea.area();
        // if (a > max_area)  -- max_area (float) promoted to double for the comparison
        if a > max_area as f64 {
            // max_area = a;  -- narrowed double->float on assignment
            max_area = a as f32;
            idx_max_area = i as i32;
        }
    }
    // if (idx_max_area >= 0) expolys_out.emplace_back(std::move(expolys_avoid[idx_max_area]));
    if idx_max_area >= 0 {
        expolys_out.push(expolys_avoid[idx_max_area as usize].clone());
    }

    expolys_out
}

// ============================================================================
// Configuration access. `PrintObjectConfig` / `PrintConfig` are the dynamic
// configurations of `crate::print::faithful`; `config.<key>.value` is read by key.
// ============================================================================

// `config.<key>.value` of a float option.
fn cfg_float(config: &DynamicPrintConfig, key: &str) -> f64 {
    config.option(key).and_then(|o| o.get_float().ok()).unwrap_or(0.0)
}

// `config.<key>.value` of an int or enum option.
fn cfg_int(config: &DynamicPrintConfig, key: &str) -> i32 {
    config.option(key).and_then(|o| o.get_int().ok()).unwrap_or(0)
}

// `config.<key>.value` of a bool option.
fn cfg_bool(config: &DynamicPrintConfig, key: &str) -> bool {
    config.option(key).and_then(|o| o.get_bool().ok()).unwrap_or(false)
}

// `config.<key>` of a ConfigOptionFloat, for the callees that take the option.
fn cfg_float_option(config: &DynamicPrintConfig, key: &str) -> ConfigOptionFloat {
    ConfigOptionFloat::with_value(cfg_float(config, key))
}

// Point.hpp `Point operator/(const Point&, coord_t)` of Eigen: integer division of both coordinates.
#[inline]
fn point_div(pt: Point, d: Coord) -> Point {
    Point::new(pt.x / d, pt.y / d)
}

// Eigen `(a - b).cast<double>().norm()`.
#[inline]
fn point_dist(a: Point, b: Point) -> f64 {
    point_norm(Point::new(a.x - b.x, a.y - b.y))
}

// ============================================================================
// TreeSupportData (TreeSupport.cpp:6707-6877)
// ============================================================================

impl TreeSupportData {
    /// TreeSupport.cpp:6707-6763
    /// `TreeSupportData::TreeSupportData(const PrintObject &object, coordf_t xy_distance, coordf_t radius_sample_resolution)`
    ///
    /// FIDELITY-NOTE: `get_bed_shape_with_excluded_area(object.print()->config())` (PrintConfig.cpp:11529)
    /// has no port in the crate; the caller hands over its result as `bed_shape_with_excluded_area`.
    /// `LayerRegion::adaptive_merge_patches` is not a member of the crate's LayerRegion, so the merged band
    /// footprints of lines 6741-6749 (multi-diameter adaptive layers) are not added to the outlines.
    pub fn new(
        object: &PrintObject,
        print: &Print,
        bed_shape_with_excluded_area: &Polygon,
        xy_distance: CoordF,
        radius_sample_resolution: CoordF,
    ) -> Self {
        let mut this = TreeSupportData { m_xy_distance: xy_distance, m_radius_sample_resolution: radius_sample_resolution, ..TreeSupportData::default() };
        // 6710 branch_scale_factor = tan(object.config().tree_support_branch_angle.value * M_PI / 180.);
        this.branch_scale_factor = (cfg_float(object.config(), "tree_support_branch_angle") * M_PI / 180.0).tan();
        this.clear_nodes();
        let n_layers = object.layers().len();
        this.m_max_move_distances.resize(n_layers, 0.0);
        this.m_layer_outlines.resize(n_layers, Vec::new());
        this.m_layer_outlines_below.resize(object.layer_count(), Vec::new());
        let mut machine_border: ExPolygons = Vec::new();
        let mut m_machine_border = ExPolygon::default();

        // 6718-6722 cal m_machine_border twice, this may happen before TreeSupport builds
        m_machine_border.contour = bed_shape_with_excluded_area.clone();
        let plate_offset = print.get_plate_origin();
        // align with the centered object in current plate (may not be the 1st plate, so need to add the plate offset)
        let shift = object.instances()[0].shift;
        let offset_pt = point_from_f64(scale_(plate_offset[0]), scale_(plate_offset[1]));
        m_machine_border.translate(Point::new(offset_pt.x - shift.x, offset_pt.y - shift.y));

        // 6724-6729
        if !m_machine_border.contour.points.is_empty() {
            let mut hole = m_machine_border.contour.clone();
            hole.reverse();
            let machine_outline =
                ExPolygon::with_holes(offset_polygon(&m_machine_border.contour, scale_(1000.0) as f32)[0].clone(), vec![hole]);
            machine_border = machine_outline.split_expoly_with_holes(scale_(1.0) as Coord, &vec![m_machine_border.clone()]);
        }

        // 6731-6762
        for layer_nr in 0..n_layers {
            let layer = object.get_layer(layer_nr as i32);
            // m_max_move_distances[layer_nr] = layer->height * branch_scale_factor;
            this.m_max_move_distances[layer_nr] = layer.height * this.branch_scale_factor;
            let mut outline: ExPolygons = Vec::with_capacity(layer.lslices.len());
            for poly in &layer.lslices {
                append(&mut outline, crate::clipper2_utils::union_ex_2(&expolygon_simplify_p(poly, scale_(this.m_radius_sample_resolution))));
            }
            append(&mut outline, machine_border.clone());

            if layer_nr == 0 {
                this.m_layer_outlines_below[layer_nr] = outline.clone();
            } else {
                let mut below = this.m_layer_outlines_below[layer_nr - 1].clone();
                below.extend(outline.iter().cloned());
                if layer_nr % 10 == 0 {
                    below = crate::clipper2_utils::union_ex_2_expolygons(&below);
                }
                this.m_layer_outlines_below[layer_nr] = below;
            }
            this.m_layer_outlines[layer_nr] = outline;
        }
        this
    }

    /// TreeSupport.cpp:6765-6774 `const ExPolygons& TreeSupportData::get_collision(coordf_t radius, size_t layer_nr) const`
    pub fn get_collision(&self, radius: CoordF, layer_nr: usize) -> Arc<ExPolygons> {
        profiler().tic();
        let radius = self.ceil_radius(radius);
        let key = (radius.to_bits(), layer_nr);
        let found = self.m_collision_cache.borrow().get(&key).cloned();
        let collision = match found {
            Some(c) => c,
            None => self.calculate_collision(radius, layer_nr),
        };
        profiler().stage_add(TreeSupportStage::StageGetCollision);
        collision
    }

    /// TreeSupport.cpp:6776-6786 `const ExPolygons& TreeSupportData::get_avoidance(coordf_t radius, size_t layer_nr, int recursions) const`
    pub fn get_avoidance(&self, radius: CoordF, layer_nr: usize, _recursions: i32) -> Arc<ExPolygons> {
        profiler().tic();
        let radius = self.ceil_radius(radius);
        let key = (radius.to_bits(), layer_nr);
        let found = self.m_avoidance_cache.borrow().get(&key).cloned();
        let avoidance = match found {
            Some(a) => a,
            None => self.calculate_avoidance(radius, layer_nr),
        };
        profiler().stage_add(TreeSupportStage::StageGetAvoidance);
        avoidance
    }

    /// TreeSupport.cpp:6788-6796 `Polygons TreeSupportData::get_contours(size_t layer_nr) const`
    pub fn get_contours(&self, layer_nr: usize) -> Polygons {
        let mut contours: Polygons = Vec::new();
        for expoly in &self.m_layer_outlines[layer_nr] {
            contours.push(expoly.contour.clone());
        }
        contours
    }

    /// TreeSupport.cpp:6798-6806 `Polygons TreeSupportData::get_contours_with_holes(size_t layer_nr) const`
    pub fn get_contours_with_holes(&self, layer_nr: usize) -> Polygons {
        let mut contours: Polygons = Vec::new();
        for expoly in &self.m_layer_outlines[layer_nr] {
            for i in 0..expoly.num_contours() {
                contours.push(contour_or_hole(expoly, i).clone());
            }
        }
        contours
    }

    /// TreeSupport.cpp:6808-6819 `SupportNode* TreeSupportData::create_node(...)`
    #[allow(clippy::too_many_arguments)]
    pub fn create_node(
        &mut self,
        position: Point,
        distance_to_top: i32,
        obj_layer_nr: i32,
        support_roof_layers_below: i32,
        to_buildplate: bool,
        parent: Option<NodeId>,
        print_z_: CoordF,
        height_: CoordF,
        dist_mm_to_top_: CoordF,
        radius_: CoordF,
    ) -> NodeId {
        // this function may be called from multiple threads, need to lock (the arena is owned here)
        let raw_ptr = self.contact_nodes.len();
        let node = SupportNode::with_parent(
            &mut self.contact_nodes,
            raw_ptr,
            position,
            distance_to_top,
            obj_layer_nr,
            support_roof_layers_below,
            to_buildplate,
            parent,
            print_z_,
            height_,
            dist_mm_to_top_,
            radius_,
        );
        self.contact_nodes.push(node);
        // if (parent) raw_ptr->movement = position - parent->position;
        if let Some(parent) = parent {
            let pp = self.contact_nodes[parent].position;
            self.contact_nodes[raw_ptr].movement = Point::new(position.x - pp.x, position.y - pp.y);
        }
        raw_ptr
    }

    /// TreeSupport.cpp:6821-6825 `void TreeSupportData::clear_nodes()`
    pub fn clear_nodes(&mut self) {
        self.contact_nodes.clear();
    }

    /// TreeSupport.cpp:6827-6837 `coordf_t TreeSupportData::ceil_radius(coordf_t radius) const`
    pub fn ceil_radius(&self, radius: CoordF) -> CoordF {
        // size_t factor = (size_t)(radius / m_radius_sample_resolution);
        let factor = (radius / self.m_radius_sample_resolution) as usize;
        // coordf_t remains = radius - m_radius_sample_resolution * factor;
        let remains = (-self.m_radius_sample_resolution).mul_add(factor as f64, radius);
        if remains > EPSILON {
            radius + self.m_radius_sample_resolution - remains
        } else {
            radius
        }
    }

    /// TreeSupport.cpp:6839-6848 `const ExPolygons& TreeSupportData::calculate_collision(const RadiusLayerPair& key) const`
    fn calculate_collision(&self, radius: CoordF, layer_nr: usize) -> Arc<ExPolygons> {
        debug_assert!(layer_nr < self.m_layer_outlines.len());
        let mut collision_areas = offset_ex(&self.m_layer_outlines[layer_nr], scale_(radius + self.m_xy_distance) as f32);
        collision_areas = expolygons_simplify(&collision_areas, scale_(self.m_radius_sample_resolution));
        // insert() keeps the element already there
        let mut cache = self.m_collision_cache.borrow_mut();
        cache.entry((radius.to_bits(), layer_nr)).or_insert_with(|| Arc::new(collision_areas)).clone()
    }

    /// TreeSupport.cpp:6850-6877 `const ExPolygons& TreeSupportData::calculate_avoidance(const RadiusLayerPair& key) const`
    fn calculate_avoidance(&self, radius: CoordF, layer_nr: usize) -> Arc<ExPolygons> {
        let mut avoidance_areas: ExPolygons = Vec::new();
        if layer_nr > 0 {
            // Avoidance for a given layer depends on all layers beneath it so could have very deep recursion depths if
            // called at high layer heights. Limit the recursion depth to N by forcing the calculation of the layer N below.
            const MAX_RECURSION_DEPTH: usize = 100;
            // Check if we would exceed the recursion limit by trying to process this layer
            if layer_nr >= MAX_RECURSION_DEPTH
                && !self.m_avoidance_cache.borrow().contains_key(&(radius.to_bits(), layer_nr - MAX_RECURSION_DEPTH))
            {
                // Force the calculation of the layer `max_recursion_depth` below our current one, ignoring the result.
                self.get_avoidance(radius, layer_nr - MAX_RECURSION_DEPTH, 0);
            }
            avoidance_areas =
                offset_ex(&self.get_avoidance(radius, layer_nr - 1, 0), scale_(-self.m_max_move_distances[layer_nr - 1]) as f32);
        }
        let collision = self.get_collision(radius, layer_nr);
        avoidance_areas.extend(collision.iter().cloned());
        avoidance_areas = union_ex(&avoidance_areas);
        let mut cache = self.m_avoidance_cache.borrow_mut();
        cache.entry((radius.to_bits(), layer_nr)).or_insert_with(|| Arc::new(avoidance_areas)).clone()
    }
}

// ============================================================================
// TreeSupport (TreeSupport.hpp:381-643, TreeSupport.cpp:605-6705)
// ============================================================================

// libslic3r.h:92 `static constexpr float g_config_tree_support_collision_resolution = 0.2;`
const G_CONFIG_TREE_SUPPORT_COLLISION_RESOLUTION: f32 = 0.2;

// Flow.cpp:293-301 `float support_nozzle_diameter(const PrintObject *object, unsigned int filament_id)`
fn support_nozzle_diameter(print: &Print, filament_id: u32) -> f32 {
    if filament_id == 0 {
        let single_nozzle = print.single_diameter_layering_nozzle();
        if single_nozzle > EPSILON {
            return single_nozzle as f32;
        }
    }
    crate::print::faithful::nozzle_diameter_for_filament(print.config(), filament_id)
}

/// A key of a `std::map<coordf_t, ...>`: ordered as the doubles are.
#[derive(Debug, Clone, Copy, PartialEq)]
struct MapKeyF64(f64);
impl Eq for MapKeyF64 {}
impl PartialOrd for MapKeyF64 {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for MapKeyF64 {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        // std::less<double>: neither below the other is the same key (0.0 and -0.0).
        if self.0 < other.0 {
            std::cmp::Ordering::Less
        } else if other.0 < self.0 {
            std::cmp::Ordering::Greater
        } else {
            std::cmp::Ordering::Equal
        }
    }
}

/// TreeSupport.hpp:516-520 `struct FoldCarrier`
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FoldCarrier {
    pub layer: usize,
    pub bottom_z: CoordF,
}

/// TreeSupport.hpp:381-643 `class TreeSupport`.
///
/// `m_object` / `m_object_config` / `m_print_config`: the object and print of `crate::print::faithful`.
/// `m_ts_data` is the `std::shared_ptr<TreeSupportData>` the object allocates
/// (`PrintObject::alloc_tree_support_preview_cache()`, PrintObject.cpp:1520-1528); the crate's PrintObject has
/// no preview cache member, so the tree support owns it. `contact_nodes` holds `NodeId`s into
/// `m_ts_data.contact_nodes`.
///
/// FIDELITY-NOTE: `SupportParameters m_support_params(object)` is built by the caller: the crate's
/// `SupportParameters::from_print_object` takes the slicing pipeline's PrintObject, not this one. Its
/// `support_base_width`, `preset_nozzle_diameter` and `multi_nozzle_diameter_mode` members
/// (SupportParameters.hpp:338-341) are not in the crate's struct and are kept here.
pub struct TreeSupport<'a> {
    // TreeSupport.hpp:424-431
    pub avg_node_per_layer: i32,
    pub nodes_angle: f32,
    pub has_sharp_tails: bool,
    pub has_cantilever: bool,
    pub max_cantilever_dist: f64,
    /// `SupportType support_type` as its integer value (`enums::SupportType`).
    pub support_type: i32,
    // TreeSupport.hpp:437-438
    pub m_print_config: &'a DynamicPrintConfig,
    pub m_machine_border: ExPolygon,
    /// TreeSupport.hpp:441 `std::map<const ExPolygon*, OverhangType> overhang_types`: not keyed by address here,
    /// the types travel in `Layer::loverhangs_with_type`.
    // TreeSupport.hpp:442
    pub m_vertical_enforcer_points: Vec<([f32; 3], [f32; 3])>,

    // TreeSupport.hpp:446-457
    pub contact_nodes: Vec<Vec<NodeId>>,
    pub m_ts_data: TreeSupportData,
    pub m_object: &'a mut PrintObject,
    pub m_print: &'a Print,
    pub m_slicing_params: SlicingParams,
    pub m_support_params: SupportParameters,
    pub m_support_merge_plan: SupportMergePlan,
    pub m_raft_layers: usize,
    pub m_highest_overhang_layer: usize,
    pub m_spanning_trees: Vec<Vec<MinimumSpanningTree>>,

    // TreeSupport.hpp:460 float DO_NOT_MOVER_UNDER_MM = 2.0;
    pub do_not_mover_under_mm: f32,
    // TreeSupport.hpp:461-465
    pub base_radius: CoordF,
    pub tip_radius: CoordF,
    // TreeSupport.hpp:472 double diameter_angle_scale_factor = tan(5.0*M_PI/180.0);
    pub diameter_angle_scale_factor: f64,
    // TreeSupport.hpp:474 const double minimum_roof_area{SQ(scaled<double>(1.))};
    pub minimum_roof_area: f64,
    // TreeSupport.hpp:486-491
    pub top_z_distance: f32,
    pub m_top_z_outlines: Vec<Vec<(f32, ExPolygons)>>,
    pub m_top_z_max: f32,
    // TreeSupport.hpp:503-505
    pub is_strong: bool,
    pub is_slim: bool,
    pub with_infill: bool,
    // TreeSupport.hpp:523-528
    pub m_fold_carriers: Vec<Vec<FoldCarrier>>,
    pub m_fold_slab_of_layer: Vec<i32>,
    pub m_base_only_fold_layers: Vec<usize>,

    /// `m_object->print()->canceled()`: the crate's Print keeps its cancel state private.
    pub canceled: Box<dyn Fn() -> bool + 'a>,
    /// SupportParameters.hpp:338 `ConfigOptionFloat support_base_width`
    pub support_base_width: ConfigOptionFloat,
    /// SupportParameters.hpp:340 `float preset_nozzle_diameter`
    pub preset_nozzle_diameter: f32,
    /// SupportParameters.hpp:341 `bool multi_nozzle_diameter_mode`
    pub multi_nozzle_diameter_mode: bool,
}

// TreeSupport.hpp:466-470, 480-484
const TS_MAX_BRANCH_RADIUS: CoordF = 10.0;
const TS_MIN_BRANCH_RADIUS: CoordF = 0.4;
/// TreeSupport.hpp:478 `const double MIN_CARRIED_RATIO = 0.3;`
pub const MIN_CARRIED_RATIO: f64 = 0.3;
/// TreeSupport.hpp:484 `const double MIN_CARRIED_RATIO_INTERFACE = 0.02;`
pub const MIN_CARRIED_RATIO_INTERFACE: f64 = 0.02;

impl<'a> TreeSupport<'a> {
    /// `m_object_config`
    fn object_config(&self) -> &DynamicPrintConfig {
        self.m_object.config()
    }

    /// TreeSupport.cpp:605-648 `TreeSupport::TreeSupport(PrintObject& object, const SlicingParameters &slicing_params)`
    ///
    /// FIDELITY-NOTE: see the struct for `support_params`; `bed_shape_with_excluded_area` is the result of
    /// `get_bed_shape_with_excluded_area(*m_print_config)` (PrintConfig.cpp:11529), which the crate has no port of.
    pub fn new(
        object: &'a mut PrintObject,
        print: &'a Print,
        slicing_params: &SlicingParams,
        support_params: SupportParameters,
        bed_shape_with_excluded_area: &Polygon,
    ) -> Self {
        let m_print_config = print.config();
        let full_print_config = print.full_print_config();
        // 609 m_raft_layers = slicing_params.base_raft_layers + slicing_params.interface_raft_layers;
        let m_raft_layers = slicing_params.base_raft_layers + slicing_params.interface_raft_layers;
        let mut m_support_params = support_params;
        let object_config = object.config();
        // 610 support_type = m_object_config->support_type;
        let support_type = cfg_int(object_config, "support_type");

        // 612-617
        let mut support_pattern = cfg_int(object_config, "support_base_pattern");
        if m_support_params.support_style == SupportMaterialStyle::SmsTreeHybrid as i32
            && support_pattern == SupportMaterialPattern::SmpDefault as i32
        {
            support_pattern = SupportMaterialPattern::SmpRectilinear as i32; //smpLightning;
        }
        if support_pattern == SupportMaterialPattern::SmpLightning as i32 {
            m_support_params.base_fill_pattern = crate::print_config::InfillPattern::Lightning;
        }
        // 618 diameter_angle_scale_factor = std::clamp<double>(tree_support_branch_diameter_angle * M_PI / 180., 0., 0.5 * M_PI - EPSILON);
        let diameter_angle_scale_factor =
            (cfg_float(object_config, "tree_support_branch_diameter_angle") * M_PI / 180.0).clamp(0.0, 0.5 * M_PI - EPSILON);
        let support_type_enum = support_type_from_i32(support_type);
        // 619-620
        let is_slim = crate::print_config::config_def::is_tree_slim(support_type_enum, support_style_from_i32(m_support_params.support_style));
        let is_strong = crate::print_config::config_def::is_tree(support_type_enum)
            && m_support_params.support_style == SupportMaterialStyle::SmsTreeStrong as i32;
        // 621-625
        let preset_nozzle_diameter = crate::print_config::config_def::primary_nozzle_diameter(full_print_config) as f32;
        let multi_nozzle_diameter_mode = crate::print_config::config_def::multi_nozzle_diameter_mode_enabled(full_print_config);
        let support_extrusion_width: CoordF = crate::flow::Flow::extrusion_width_for_nozzle(
            &cfg_float_option(object_config, "line_width"),
            preset_nozzle_diameter,
            support_nozzle_diameter(print, cfg_int(object_config, "support_filament") as u32),
            multi_nozzle_diameter_mode,
        );
        // 626-630 A branch may not start thinner than a single extrusion, or draw_circles() is handed a circle it
        // cannot extrude, and it may not start thicker than the branch it grows into.
        let branch_diameter = cfg_float(object_config, "tree_support_branch_diameter");
        let tip_diameter_max = support_extrusion_width.max(branch_diameter);
        // std::clamp(v, lo, hi): (v < lo) ? lo : (hi < v) ? hi : v
        let tip_diameter = cfg_float(object_config, "tree_support_tip_diameter");
        let tip_radius = (if tip_diameter < support_extrusion_width {
            support_extrusion_width
        } else if tip_diameter_max < tip_diameter {
            tip_diameter_max
        } else {
            tip_diameter
        }) / 2.0;
        let base_radius = tip_radius.max(branch_diameter / 2.0);
        // 632 by default tree support needs no infill, unless it's tree hybrid which contains normal nodes.
        let with_infill = m_support_params.support_base_pattern != SupportMaterialPattern::SmpNone as i32;
        // 633-636
        let mut m_machine_border = ExPolygon::default();
        m_machine_border.contour = bed_shape_with_excluded_area.clone();
        let plate_offset = print.get_plate_origin();
        // align with the centered object in current plate (may not be the 1st plate, so need to add the plate offset)
        let shift = object.instances()[0].shift;
        let offset_pt = point_from_f64(scale_(plate_offset[0]), scale_(plate_offset[1]));
        m_machine_border.translate(Point::new(offset_pt.x - shift.x, offset_pt.y - shift.y));
        // 637 m_ts_data = m_object->alloc_tree_support_preview_cache();  (PrintObject.cpp:1520-1528)
        let xy_distance = cfg_float(object_config, "support_object_xy_distance");
        let m_ts_data = TreeSupportData::new(
            object,
            print,
            bed_shape_with_excluded_area,
            xy_distance,
            G_CONFIG_TREE_SUPPORT_COLLISION_RESOLUTION as f64,
        );
        // 638-640
        let mut top_z_distance = cfg_float(object_config, "support_top_z_distance") as f32;
        if top_z_distance as f64 > EPSILON {
            top_z_distance = top_z_distance.max(slicing_params.min_layer_height as f32);
        }
        // SupportParameters.hpp:123
        let support_base_width = if cfg_float(object_config, "support_line_width") > 0.0 {
            cfg_float_option(object_config, "support_line_width")
        } else {
            cfg_float_option(object_config, "line_width")
        };
        let mut this = TreeSupport {
            avg_node_per_layer: 0,
            nodes_angle: 0.0,
            has_sharp_tails: false,
            has_cantilever: false,
            max_cantilever_dist: 0.0,
            support_type,
            m_print_config,
            m_machine_border,
            m_vertical_enforcer_points: Vec::new(),
            contact_nodes: Vec::new(),
            m_ts_data,
            m_object: object,
            m_print: print,
            m_slicing_params: slicing_params.clone(),
            m_support_params,
            m_support_merge_plan: SupportMergePlan::default(),
            m_raft_layers,
            m_highest_overhang_layer: 0,
            m_spanning_trees: Vec::new(),
            do_not_mover_under_mm: 2.0,
            base_radius,
            tip_radius,
            diameter_angle_scale_factor,
            minimum_roof_area: sq(scale_(1.0)),
            top_z_distance,
            m_top_z_outlines: Vec::new(),
            m_top_z_max: 0.0,
            is_strong,
            is_slim,
            with_infill,
            m_fold_carriers: Vec::new(),
            m_fold_slab_of_layer: Vec::new(),
            m_base_only_fold_layers: Vec::new(),
            canceled: Box::new(|| false),
            support_base_width,
            preset_nozzle_diameter,
            multi_nozzle_diameter_mode,
        };
        // 644
        this.init_top_z_by_nozzle();
        this
    }

    /// TreeSupport.hpp:410-422 `SupportNode* create_node(...)`
    #[allow(clippy::too_many_arguments)]
    pub fn create_node(
        &mut self,
        position: Point,
        distance_to_top: i32,
        obj_layer_nr: i32,
        support_roof_layers_below: i32,
        to_buildplate: bool,
        parent: Option<NodeId>,
        print_z_: CoordF,
        height_: CoordF,
        dist_mm_to_top_: CoordF,
        radius_: CoordF,
    ) -> NodeId {
        self.m_ts_data.create_node(
            position,
            distance_to_top,
            obj_layer_nr,
            support_roof_layers_below,
            to_buildplate,
            parent,
            print_z_,
            height_,
            dist_mm_to_top_,
            radius_,
        )
    }

    /// TreeSupport.cpp:650-712 `void TreeSupport::init_top_z_by_nozzle()`
    ///
    /// FIDELITY-NOTE: the tbb::parallel_for over the layers runs in order (every iteration writes its own
    /// element). `layerm->adaptive_merge_patches` (675-676) is not a member of the crate's LayerRegion.
    pub fn init_top_z_by_nozzle(&mut self) {
        self.m_top_z_outlines.clear();
        self.m_top_z_max = self.top_z_distance;
        // The contact nodes are pre-split on the object layer grid, see generate_contact_points(), so independent
        // support layers keep the global gap. A zero gap is the soluble interface contract and is never replaced.
        if (self.top_z_distance as f64) < EPSILON
            || self.m_support_params.independent_layer_height
            || !self.m_print.uses_multiple_nozzle_diameters()
        {
            return;
        }

        let print_config = self.m_print.config();
        let min_gap = self.m_slicing_params.min_layer_height as f32;
        // const coord_t resolution = scale_(m_ts_data->m_radius_sample_resolution);
        let resolution = scale_(self.m_ts_data.m_radius_sample_resolution) as Coord;
        // An overhang's underside is printed by the walls and the bridge infill, take the larger gap of the two.
        let region_gap = |wall_filament: u32, solid_infill_filament: u32| -> f32 {
            let nozzle: f64 = (crate::print::faithful::nozzle_diameter_for_filament(print_config, wall_filament)
                .max(crate::print::faithful::nozzle_diameter_for_filament(print_config, solid_infill_filament)))
                as f64;
            crate::adaptive_layer_merge::support_top_z_for_nozzle(nozzle).max(min_gap)
        };

        let layer_count = self.m_object.layer_count();
        self.m_top_z_outlines.resize(layer_count, Vec::new());
        for layer_nr in 0..layer_count {
            let mut buckets: Vec<(f32, ExPolygons)> = Vec::new();
            // Same footprint as TreeSupportData::m_layer_outlines: the region slices. Union before simplifying, so
            // neighbouring surfaces of one region cannot open cracks between them.
            for layerm in self.m_object.get_layer(layer_nr as i32).regions() {
                let region_area: ExPolygons = crate::surface::to_expolygons(&layerm.get_slices().surfaces);
                if region_area.is_empty() {
                    continue;
                }
                let gap = region_gap(layerm.region().config().wall_filament as u32, layerm.region().config().solid_infill_filament as u32);
                // bucket_of(gap)
                let pos = match buckets.iter().position(|bucket| ((bucket.0 - gap).abs() as f64) < EPSILON) {
                    Some(pos) => pos,
                    None => {
                        buckets.push((gap, Vec::new()));
                        buckets.len() - 1
                    }
                };
                for poly in &union_ex(&region_area) {
                    append(&mut buckets[pos].1, crate::clipper2_utils::union_ex_2(&expolygon_simplify_p(poly, resolution as f64)));
                }
            }
            for bucket in buckets.iter_mut() {
                bucket.1 = union_ex(&bucket.1);
            }
            buckets.retain(|bucket| !bucket.1.is_empty());
            // std::sort by the gap: the gaps of the buckets differ by EPSILON or more, so no two compare
            // equal and any sort gives the one order.
            buckets.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
            self.m_top_z_outlines[layer_nr] = buckets;
        }

        let mut max_gap: f32 = 0.0;
        for buckets in &self.m_top_z_outlines {
            if !buckets.is_empty() {
                max_gap = max_gap.max(buckets[buckets.len() - 1].0);
            }
        }
        if max_gap <= 0.0 {
            self.m_top_z_outlines.clear();
        } else {
            self.m_top_z_max = max_gap;
        }
    }

    /// TreeSupport.cpp:714-721 `float TreeSupport::top_z_at(size_t obj_layer_nr) const`
    pub fn top_z_at(&self, obj_layer_nr: usize) -> f32 {
        if self.m_top_z_outlines.is_empty() {
            return self.top_z_distance;
        }
        if obj_layer_nr >= self.m_top_z_outlines.len() || self.m_top_z_outlines[obj_layer_nr].is_empty() {
            return self.m_top_z_max;
        }
        let buckets = &self.m_top_z_outlines[obj_layer_nr];
        buckets[buckets.len() - 1].0
    }

    /// TreeSupport.cpp:723-741 `float TreeSupport::top_z_for_overhang(size_t obj_layer_nr, const ExPolygon &overhang) const`
    pub fn top_z_for_overhang(&self, obj_layer_nr: usize, overhang: &ExPolygon) -> f32 {
        if self.m_top_z_outlines.is_empty() || obj_layer_nr >= self.m_top_z_outlines.len() || self.m_top_z_outlines[obj_layer_nr].len() < 2 {
            return self.top_z_at(obj_layer_nr);
        }
        // The outlines are simplified, so a neighbouring region's may poke a sliver into the overhang. Only an
        // overlap wider than that error counts; a mere touch is the fallback when nothing wider is found.
        let sliver = scale_(self.m_ts_data.m_radius_sample_resolution) as f32;
        let mut gap: f32 = 0.0;
        let mut touched: f32 = 0.0;
        for bucket in &self.m_top_z_outlines[obj_layer_nr] {
            let common = intersection_ex(&bucket.1, std::slice::from_ref(overhang));
            if common.is_empty() {
                continue;
            }
            touched = touched.max(bucket.0);
            if !opening_ex(&common, sliver).is_empty() {
                gap = gap.max(bucket.0);
            }
        }
        if gap > 0.0 {
            gap
        } else if touched > 0.0 {
            touched
        } else {
            self.top_z_at(obj_layer_nr)
        }
    }

    /// TreeSupport.cpp:743-776
    /// `void TreeSupport::union_outlines_within_top_z(ExPolygons &collision, size_t obj_layer_nr, bool clamp_to_first_layer_gap) const`
    pub fn union_outlines_within_top_z(&self, collision: &mut ExPolygons, obj_layer_nr: usize, clamp_to_first_layer_gap: bool) {
        if self.top_z_distance as f64 <= EPSILON {
            return;
        }
        // The object first layer is already offset by support_object_first_layer_gap. Outlines above still
        // have to be cleared, but expanding them by the full top-z gap cancels a smaller first-layer gap on a
        // vertical wall (the upper outline matches this one). Clamp only the XY expansion; the Z window stays
        // the top-z gap so a real overhang within that height is still kept out.
        let first_layer_gap: f32 =
            if clamp_to_first_layer_gap { cfg_float(self.object_config(), "support_object_first_layer_gap") as f32 } else { 0.0 };
        // returns scale_(gap): a double, narrowed to the float delta of offset_ex
        let xy_expand = |mut gap: f32| -> f32 {
            if clamp_to_first_layer_gap {
                gap = gap.min(first_layer_gap);
            }
            scale_(gap as f64) as f32
        };
        let mut above: ExPolygons = Vec::new();
        let mut accum_height: f32 = 0.0;
        for layer_id in obj_layer_nr + 1..self.m_ts_data.m_layer_outlines.len() {
            // accum_height += m_object->get_layer(layer_id)->height;  (float += double)
            accum_height = (accum_height as f64 + self.m_object.get_layer(layer_id as i32).height) as f32;
            if accum_height > self.m_top_z_max {
                break;
            }
            if self.m_top_z_outlines.is_empty() {
                append(&mut above, offset_ex(&self.m_ts_data.m_layer_outlines[layer_id], xy_expand(self.top_z_distance)));
            } else {
                // A fine nozzle's own overhang right above its thin gap must not be pulled down onto its interface
                // because a coarse nozzle elsewhere asks for a wider gap.
                for bucket in &self.m_top_z_outlines[layer_id] {
                    if accum_height <= bucket.0 {
                        append(&mut above, offset_ex(&bucket.1, xy_expand(bucket.0)));
                    }
                }
            }
        }
        if !above.is_empty() {
            append(collision, above);
            *collision = union_ex(collision);
        }
    }
}

/// TreeSupport.cpp:778-782 `void add_overhang(Layer *layer, const ExPolygon &overhang, int type)`
pub fn add_overhang(layer: &mut Layer, overhang: &ExPolygon, type_: i32) {
    layer.loverhangs_with_type.push((overhang.clone(), type_));
    layer.loverhangs.push(overhang.clone());
}

// PrintConfig.hpp `SupportType` / `SupportMaterialStyle` of their integer values.
fn support_type_from_i32(v: i32) -> SupportType {
    match v {
        0 => SupportType::StNormalAuto,
        1 => SupportType::StTreeAuto,
        2 => SupportType::StNormal,
        _ => SupportType::StTree,
    }
}

fn support_style_from_i32(v: i32) -> SupportMaterialStyle {
    match v {
        0 => SupportMaterialStyle::SmsDefault,
        1 => SupportMaterialStyle::SmsGrid,
        2 => SupportMaterialStyle::SmsSnug,
        3 => SupportMaterialStyle::SmsTreeSlim,
        4 => SupportMaterialStyle::SmsTreeStrong,
        5 => SupportMaterialStyle::SmsTreeHybrid,
        _ => SupportMaterialStyle::SmsTreeOrganic,
    }
}

// Polygon.cpp:171-187 `Point Polygon::centroid() const`. The result is `Point(Vec2d(...))`, the explicit
// constructor of Point.hpp:181, which rounds with lrint. `cross2` (Point.hpp) is one expression,
// `v1.x() * v2.y() - v2.x() * v1.y()`.
pub fn polygon_centroid(poly: &Polygon) -> Point {
    let points = &poly.points;
    let mut area_sum = 0.0_f64;
    let mut c = (0.0_f64, 0.0_f64);
    if points.len() >= 3 {
        let back = points[points.len() - 1];
        let mut p1 = (back.x as f64, back.y as f64);
        for p in points {
            let p2 = (p.x as f64, p.y as f64);
            let a = p1.0.mul_add(p2.1, -(p2.0 * p1.1));
            area_sum += a;
            c.0 += (p1.0 + p2.0) * a;
            c.1 += (p1.1 + p2.1) * a;
            p1 = p2;
        }
    }
    point_from_f64(c.0 / (3.0 * area_sum), c.1 / (3.0 * area_sum))
}

// Point.hpp:226-231 `void Point::rotate(double cos_a, double sin_a)`
#[inline]
fn point_rotate(pt: Point, cos_a: f64, sin_a: f64) -> Point {
    let cur_x = pt.x as f64;
    let cur_y = pt.y as f64;
    Point::new(cos_a.mul_add(cur_x, -(sin_a * cur_y)).round() as Coord, cos_a.mul_add(cur_y, sin_a * cur_x).round() as Coord)
}

/// TreeSupport.cpp:1758-1784 `static ExPolygon tree_support_node_disk(const SupportNode &node, const Polygon &unit_circle,
/// coordf_t branch_radius, coordf_t branch_radius_scaled, bool square_support, coordf_t radius = 0.)`
pub fn tree_support_node_disk(
    node: &SupportNode,
    unit_circle: &Polygon,
    branch_radius: CoordF,
    branch_radius_scaled: CoordF,
    square_support: bool,
    radius: CoordF,
) -> ExPolygon {
    let mut circle = unit_circle.clone();
    let scale = (if radius > 0.0 { radius } else { node.radius }) / branch_radius;
    let move_x = node.movement.x as f64 / (scale * branch_radius_scaled);
    let move_y = node.movement.y as f64 / (scale * branch_radius_scaled);
    if !square_support && move_x.abs() > 0.001 && move_y.abs() > 0.001 {
        let vsize_inv = 0.5 / (0.01 + move_x.mul_add(move_x, move_y * move_y).sqrt());
        let matrix: [f64; 4] = [
            scale * (move_x * move_x).mul_add(vsize_inv, 1.0),
            scale * move_x * move_y * vsize_inv,
            scale * move_x * move_y * vsize_inv,
            scale * (move_y * move_y).mul_add(vsize_inv, 1.0),
        ];
        for vertex in circle.points.iter_mut() {
            let x = matrix[0].mul_add(vertex.x as f64, matrix[1] * vertex.y as f64);
            let y = matrix[2].mul_add(vertex.x as f64, matrix[3] * vertex.y as f64);
            let p = point_from_f64(x, y);
            *vertex = Point::new(node.position.x + p.x, node.position.y + p.y);
        }
    } else {
        for vertex in circle.points.iter_mut() {
            // vertex = vertex * scale + node.position;  the member operator* of a non-const Point (lrint)
            let p = point_mul(*vertex, scale);
            *vertex = Point::new(p.x + node.position.x, p.y + node.position.y);
        }
    }
    ExPolygon::new(circle)
}

/// TreeSupport.cpp:4613-4625 `static double carried_area(const ExPolygons &carrier,
/// const std::vector<BoundingBox> &carrier_bboxes, const ExPolygon &region)`
///
/// Area of a region that the carrier below holds up. The bounding boxes come from get_extents_vector(carrier)
/// and spare the clipper boolean the candidates that are nowhere near the region.
pub fn carried_area(carrier: &ExPolygons, carrier_bboxes: &[BoundingBox], region: &ExPolygon) -> f64 {
    if carrier.is_empty() {
        return 0.0;
    }
    let region_bbox = get_extents_expoly(region);
    let mut near_carrier: Polygons = Vec::new();
    for i in 0..carrier.len() {
        if bbox_overlap(&carrier_bboxes[i], &region_bbox) {
            near_carrier.extend(crate::clipper_utils::clip_clipper_polygons_with_subject_bbox_expolygon(&carrier[i], &region_bbox, false));
        }
    }
    if near_carrier.is_empty() {
        return 0.0;
    }
    let mut area = 0.0;
    for part in &intersection_ex_polygons(std::slice::from_ref(region), &near_carrier) {
        area += part.area();
    }
    area
}

/// TreeSupport.cpp:4628-4633 `static double carried_area_ratio(...)`: fraction of a region that the carrier below holds up.
pub fn carried_area_ratio(carrier: &ExPolygons, carrier_bboxes: &[BoundingBox], region: &ExPolygon) -> f64 {
    let region_area = region.area();
    if region_area <= 0.0 {
        return 0.0;
    }
    carried_area(carrier, carrier_bboxes, region) / region_area
}

// BoundingBox.hpp:57-60 `bool overlap(const BoundingBoxBase<PointClass> &other) const`
#[inline]
fn bbox_overlap(a: &BoundingBox, b: &BoundingBox) -> bool {
    !(a.max.x < b.min.x || a.min.x > b.max.x || a.max.y < b.min.y || a.min.y > b.max.y)
}

/// TreeSupport.cpp:5103-5122 `static void smooth_filter(const Polygon &poly, int filter_size,
/// std::unordered_map<Point, Point, PointHash> &movements, double max_move)`
pub fn smooth_filter(poly: &Polygon, filter_size: i32, movements: &mut HashMap<Point, Point>, max_move: f64) {
    let pts = &poly.points;
    let n = pts.len() as i32;
    let center = polygon_centroid(poly);
    let mut newpoly = poly.clone();
    for i in 0..n {
        let mut filter_pt = Point::new(0, 0);
        let mut j = (i + n - filter_size) % n;
        while j != (i + filter_size + 1) % n {
            filter_pt = Point::new(filter_pt.x + pts[j as usize].x, filter_pt.y + pts[j as usize].y);
            j = (j + 1) % n;
        }
        let avg = point_div(filter_pt, (2 * filter_size + 1) as Coord);
        newpoly.points[i as usize] = Point::new(avg.x - center.x, avg.y - center.y);
    }
    if newpoly.area() < sq(scale_(10.0)) {
        // MultiPoint.cpp:6-10 `void MultiPoint::scale(double factor)`: `pt *= factor`, Point.hpp:199 (truncates)
        let factor = sq(scale_(10.0)) / newpoly.area();
        for pt in newpoly.points.iter_mut() {
            *pt = Point::new((pt.x as f64 * factor) as Coord, (pt.y as f64 * factor) as Coord);
        }
    }
    for i in 0..n as usize {
        let pt = pts[i];
        let mut move_ = Point::new(newpoly.points[i].x - (pt.x - center.x), newpoly.points[i].y - (pt.y - center.y));
        if vsize2_with_unscale(move_) > sq(max_move) {
            move_ = normal(move_, scale_(max_move));
        }
        movements.insert(pt, move_);
    }
}

/// TreeSupport.cpp:5124-5145 `static void densify_polygon(Polygon &poly, double max_edge_length)`
pub fn densify_polygon(poly: &mut Polygon, max_edge_length: f64) {
    let mut dense: Vec<Point> = Vec::new();
    let pts = &poly.points;
    let n = pts.len();
    for i in 0..n {
        let i2 = (i + 1) % n;
        dense.push(pts[i]);
        let mut prev = pts[i];
        // auto edge = pts[i2] - pts[i];
        let edge = Point::new(pts[i2].x - pts[i].x, pts[i2].y - pts[i].y);
        let n_frag = (vsize2_with_unscale(edge).sqrt() / max_edge_length).ceil() as i32;
        for j in 1..n_frag {
            // curr = pts[i] + edge * (double(j) / double(n_frag));  `edge` is a Point (Point.hpp `operator-`), not
            // const: the member operator* (lrint)
            let step = point_mul(edge, j as f64 / n_frag as f64);
            let curr = Point::new(pts[i].x + step.x, pts[i].y + step.y);
            if curr != prev {
                prev = curr;
                dense.push(curr);
            }
        }
    }
    poly.points = dense;
}

impl<'a> TreeSupport<'a> {
    /// TreeSupport.cpp:1554-1596 `void TreeSupport::create_tree_support_layers()`
    pub fn create_tree_support_layers(&mut self) {
        let mut layer_id: i32 = 0;
        if self.m_raft_layers > 0 {
            //create raft layers
            let mut raft_print_z: CoordF = 0.0;
            let mut raft_slice_z: CoordF;
            if self.m_slicing_params.base_raft_layers > 0 {
                // Do not add the raft contact layer, 1st layer should use first_print_layer_height
                let height = self.m_slicing_params.first_print_layer_height;
                raft_print_z += height;
                raft_slice_z = raft_print_z - height / 2.0;
                self.m_object.add_tree_support_layer(layer_id, height, raft_print_z, raft_slice_z);
                layer_id += 1;
            }
            // Insert the base layers.
            for _i in 1..self.m_slicing_params.base_raft_layers {
                let height = self.m_slicing_params.base_raft_layer_height;
                raft_print_z += height;
                raft_slice_z = raft_print_z - height / 2.0;
                self.m_object.add_tree_support_layer(layer_id, height, raft_print_z, raft_slice_z);
                layer_id += 1;
            }
            // Insert the interface layers.
            for _i in 0..self.m_slicing_params.interface_raft_layers {
                let height = self.m_slicing_params.interface_raft_layer_height;
                raft_print_z += height;
                raft_slice_z = raft_print_z - height / 2.0;
                self.m_object.add_tree_support_layer(layer_id, height, raft_print_z, raft_slice_z);
                layer_id += 1;
            }
            // Layers between the raft contacts and bottom of the object.
            let dist_to_go = self.m_slicing_params.object_print_z_min - raft_print_z;
            if dist_to_go > EPSILON {
                let nsteps = (dist_to_go / self.m_slicing_params.max_suport_layer_height).ceil() as i32;
                let height = dist_to_go / nsteps as f64;
                for _i in 0..nsteps {
                    raft_print_z += height;
                    raft_slice_z = raft_print_z - height / 2.0;
                    self.m_object.add_tree_support_layer(layer_id, height, raft_print_z, raft_slice_z);
                    layer_id += 1;
                }
            }
            self.m_raft_layers = layer_id as usize;
        }
    }

    /// TreeSupport.cpp:1745-1756 `coordf_t TreeSupport::support_flare_step_width(coordf_t print_z, coordf_t layer_height) const`
    pub fn support_flare_step_width(&self, print_z: CoordF, layer_height: CoordF) -> CoordF {
        let mut band_height = 0.0_f64;
        let candidate = self.m_support_merge_plan.coarsest_band_containing(print_z, &mut band_height);
        if !candidate.valid() || band_height <= layer_height + EPSILON {
            return self.m_support_params.support_extrusion_width;
        }
        // const Flow slab_flow = m_support_params.support_material_flow_for_nozzle(candidate.nozzle_diameter, float(band_height));
        // SupportParameters.hpp:342-346 -> Flow::new_from_config_width (Flow.cpp:136-157); only the width is read:
        // `float w`, the automatic width when the option is 0, else extrusion_width_for_nozzle().
        let nozzle_diameter = candidate.nozzle_diameter as f32;
        let w: f32 = if self.support_base_width.value == 0.0 {
            crate::flow::Flow::auto_extrusion_width(crate::flow::FlowRole::SupportMaterial, nozzle_diameter as f64) as f32
        } else {
            crate::flow::Flow::extrusion_width_for_nozzle(
                &self.support_base_width,
                self.preset_nozzle_diameter,
                nozzle_diameter,
                self.multi_nozzle_diameter_mode,
            ) as f32
        };
        // return slab_flow.width() * layer_height / band_height;
        w as f64 * layer_height / band_height
    }

    /// TreeSupport.cpp:2978-2988 `std::vector<size_t> TreeSupport::fold_carriers_above(size_t layer_nr) const`
    pub fn fold_carriers_above(&self, layer_nr: usize) -> Vec<usize> {
        let mut carriers: Vec<usize> = Vec::new();
        if layer_nr >= self.m_fold_slab_of_layer.len() || self.m_fold_slab_of_layer[layer_nr] < 0 {
            return carriers;
        }
        let layer_top = self.m_ts_data.layer_heights[layer_nr].print_z;
        for carrier in &self.m_fold_carriers[self.m_fold_slab_of_layer[layer_nr] as usize] {
            if carrier.layer > layer_nr && layer_top > carrier.bottom_z + EPSILON {
                carriers.push(carrier.layer);
            }
        }
        carriers
    }

    /// TreeSupport.cpp:3702-3720
    /// `coordf_t TreeSupport::calc_branch_radius(coordf_t base_radius, size_t layers_to_top, size_t tip_layers, double diameter_angle_scale_factor)`
    pub fn calc_branch_radius_layers(&self, base_radius: CoordF, layers_to_top: usize, tip_layers: usize, diameter_angle_scale_factor: f64) -> CoordF {
        let radius: f64;
        if !self.is_slim {
            if (layers_to_top + 1) > tip_layers {
                radius = (base_radius * (layers_to_top + 1) as f64).mul_add(diameter_angle_scale_factor, base_radius);
            } else {
                radius = base_radius * (layers_to_top + 1) as f64 / tip_layers as f64;
            }
        } else if (layers_to_top + 1) > tip_layers * 2 {
            radius = (base_radius * (layers_to_top + 1) as f64).mul_add(diameter_angle_scale_factor, base_radius);
        } else {
            radius = base_radius * (layers_to_top + 1) as f64 / (tip_layers * 2) as f64;
        }
        // radius = std::clamp(radius, tip_radius, MAX_BRANCH_RADIUS);
        std_clamp(radius, self.tip_radius, TS_MAX_BRANCH_RADIUS)
    }

    /// TreeSupport.cpp:3722-3743
    /// `coordf_t TreeSupport::calc_branch_radius(coordf_t base_radius, coordf_t mm_to_top, double diameter_angle_scale_factor, bool use_min_distance)`
    pub fn calc_branch_radius(&self, base_radius: CoordF, mm_to_top: CoordF, diameter_angle_scale_factor: f64, _use_min_distance: bool) -> CoordF {
        let mut radius: f64;
        let tip_height = base_radius; // this is a 45 degree tip
        if mm_to_top > tip_height {
            radius = (mm_to_top - tip_height).mul_add(diameter_angle_scale_factor, base_radius);
        } else {
            radius = mm_to_top; // this is a 45 degree tip
        }
        radius = std_clamp(radius, self.tip_radius, TS_MAX_BRANCH_RADIUS);
        if cfg_int(self.object_config(), "support_interface_top_layers") > 0 {
            radius = radius.max(base_radius);
        }
        // #if USE_SUPPORT_3D ... #endif
        radius
    }

    /// TreeSupport.cpp:3745-3748 `coordf_t TreeSupport::calc_radius(coordf_t mm_to_top)`
    pub fn calc_radius(&self, mm_to_top: CoordF) -> CoordF {
        self.calc_branch_radius(self.base_radius, mm_to_top, self.diameter_angle_scale_factor, true)
    }

    /// TreeSupport.cpp:3750-3755 `coordf_t TreeSupport::get_radius(const SupportNode* node)`
    pub fn get_radius(&mut self, node: NodeId) -> CoordF {
        if self.m_ts_data.contact_nodes[node].radius == 0.0 {
            let radius = self.calc_radius(self.m_ts_data.contact_nodes[node].dist_mm_to_top);
            self.m_ts_data.contact_nodes[node].radius = radius;
        }
        self.m_ts_data.contact_nodes[node].radius
    }

    /// TreeSupport.cpp:3757-3772 `ExPolygons TreeSupport::get_avoidance(coordf_t radius, size_t obj_layer_nr)` (USE_SUPPORT_3D 0)
    pub fn get_avoidance(&self, radius: CoordF, obj_layer_nr: usize) -> Arc<ExPolygons> {
        self.m_ts_data.get_avoidance(radius, obj_layer_nr, 0)
    }

    /// TreeSupport.cpp:3774-3789 `ExPolygons TreeSupport::get_collision(coordf_t radius, size_t layer_nr)` (USE_SUPPORT_3D 0)
    pub fn get_collision(&self, radius: CoordF, layer_nr: usize) -> Arc<ExPolygons> {
        self.m_ts_data.get_collision(radius, layer_nr)
    }

    /// TreeSupport.cpp:3791-3808 `Polygons TreeSupport::get_collision_polys(coordf_t radius, size_t layer_nr)` (USE_SUPPORT_3D 0)
    pub fn get_collision_polys(&self, radius: CoordF, layer_nr: usize) -> Polygons {
        let expolys = self.m_ts_data.get_collision(radius, layer_nr);
        let mut polys: Polygons = Vec::new();
        for expoly in expolys.iter() {
            for i in 0..expoly.num_contours() {
                polys.push(contour_or_hole(expoly, i).clone());
            }
        }
        polys
    }

    /// TreeSupport.cpp:6036-6136 `void TreeSupport::smooth_nodes()` (USE_TREE_SUPPORT_CARRIED_MASS 1)
    pub fn smooth_nodes(&mut self) {
        for layer_nr in 0..self.contact_nodes.len() {
            if self.contact_nodes[layer_nr].is_empty() {
                continue;
            }
            for &node in &self.contact_nodes[layer_nr] {
                self.m_ts_data.contact_nodes[node].is_processed = false;
            }
        }
        // 6045-6053
        let support_nozzle: CoordF = support_nozzle_diameter(self.m_print, cfg_int(self.object_config(), "support_filament") as u32) as f64;
        let support_lw: CoordF = if cfg_float(self.object_config(), "support_line_width") > 0.0 {
            crate::flow::Flow::extrusion_width_for_nozzle(
                &cfg_float_option(self.object_config(), "support_line_width"),
                crate::print_config::config_def::primary_nozzle_diameter(self.m_print.full_print_config()) as f32,
                support_nozzle as f32,
                crate::print_config::config_def::multi_nozzle_diameter_mode_enabled(self.m_print.full_print_config()),
            )
        } else {
            self.m_support_params.support_extrusion_width
        };
        let max_move: f32 = scale_(support_lw / 2.0) as f32;
        // if the branch is very tall, the tip also needs extra wall
        let thresh_tall_branch: f32 = 100.0;
        let thresh_dist_to_top: f32 = 30.0;
        let nodes = &mut self.m_ts_data.contact_nodes;
        for layer_nr in 0..self.contact_nodes.len() {
            if self.contact_nodes[layer_nr].is_empty() {
                continue;
            }
            for &node in &self.contact_nodes[layer_nr] {
                if !nodes[node].is_processed {
                    let mut pts: Vec<Point> = Vec::new();
                    let mut radii: Vec<f64> = Vec::new();
                    let mut branch: Vec<NodeId> = Vec::new();
                    let mut p_node: Option<NodeId> = Some(node);
                    let mut total_height: f32 = 0.0;
                    // add a fixed head if it's not a polygon node, see STUDIO-4403
                    // Polygon node can't be added because the move distance might be huge, making the nodes in between jump and dangling
                    if let Some(child) = nodes[node].child {
                        if nodes[child].type_ != TreeNodeType::EPolygon {
                            pts.push(nodes[child].position);
                            radii.push(nodes[child].radius);
                            branch.push(child);
                            total_height = (total_height as f64 + nodes[child].height) as f32;
                        }
                    }
                    loop {
                        let p = p_node.unwrap();
                        pts.push(nodes[p].position);
                        radii.push(nodes[p].radius);
                        branch.push(p);
                        total_height = (total_height as f64 + nodes[p].height) as f32;
                        p_node = nodes[p].parent;
                        if !(p_node.is_some() && !nodes[p_node.unwrap()].is_processed) {
                            break;
                        }
                    }
                    if pts.len() < 3 {
                        continue;
                    }

                    let mut pts1 = pts.clone();
                    let mut radii1 = radii.clone();
                    // TODO here we assume layer height gap is constant. If not true, need to consider height jump
                    const ITERATIONS: usize = 100;
                    for k in 0..ITERATIONS {
                        for i in 1..pts.len() - 1 {
                            let pt = point_div(
                                Point::new(pts[i - 1].x + pts[i].x + pts[i + 1].x, pts[i - 1].y + pts[i].y + pts[i + 1].y),
                                3,
                            );
                            pts1[i] = pt;
                            radii1[i] = (radii[i - 1] + radii[i] + radii[i + 1]) / 3.0;
                            if k == ITERATIONS - 1 {
                                let b = &mut nodes[branch[i]];
                                b.position = pt;
                                b.radius = radii1[i];
                                b.movement = point_div(Point::new(pts[i + 1].x - pts[i - 1].x, pts[i + 1].y - pts[i - 1].y), 2);
                                b.is_processed = true;
                                b.overhang_degree = (10.0_f64).min(point_norm(b.movement) * SCALING_FACTOR / support_lw * 10.0);
                                if (b.movement.x as f32 > max_move || b.movement.y as f32 > max_move)
                                    || (total_height > thresh_tall_branch && b.dist_mm_to_top < thresh_dist_to_top as f64)
                                {
                                    b.need_extra_wall = true;
                                }
                            }
                        }
                        if k < ITERATIONS - 1 {
                            std::mem::swap(&mut pts, &mut pts1);
                            std::mem::swap(&mut radii, &mut radii1);
                        }
                    }
                }
            }
        }
    }

    /// TreeSupport.cpp:6257-6387 `std::vector<LayerHeightData> TreeSupport::plan_layer_heights()`
    pub fn plan_layer_heights(&mut self) -> Vec<LayerHeightData> {
        let mut layer_heights: Vec<LayerHeightData> = Vec::new();
        let mut z_heights: BTreeMap<MapKeyF64, CoordF> = BTreeMap::new(); // print_z:height
        if !self.m_support_params.independent_layer_height {
            layer_heights.resize(self.m_object.layer_count(), LayerHeightData::default());
            for layer_nr in 0..self.m_object.layer_count() {
                let layer = self.m_object.get_layer(layer_nr as i32);
                z_heights.insert(MapKeyF64(layer.print_z), layer.height);
                layer_heights[layer_nr] = LayerHeightData::from_values(layer.print_z, layer.height, layer_nr);
            }
        } else {
            let max_layer_height = self.m_slicing_params.max_suport_layer_height;
            let min_layer_height = self.m_slicing_params.min_layer_height;
            let mut bounds: BTreeMap<MapKeyF64, CoordF> = BTreeMap::new(); // print_z: height
            // Keep first layer still
            let layer0_print_z = self.m_object.get_layer(0).print_z;
            let layer0_height = self.m_object.get_layer(0).height;
            bounds.insert(MapKeyF64(layer0_print_z), layer0_height);
            let mut obj_layer_zs: Vec<f32> = Vec::with_capacity(self.m_object.layer_count());
            for l in self.m_object.layers() {
                obj_layer_zs.push(l.print_z as f32);
            }
            z_heights.insert(MapKeyF64(layer0_print_z), layer0_height);
            // Collect top contact layers
            for layer_nr in 1..self.contact_nodes.len() {
                if !self.contact_nodes[layer_nr].is_empty() {
                    let front = &self.m_ts_data.contact_nodes[self.contact_nodes[layer_nr][0]];
                    let print_z = front.print_z;
                    let height = front.height;
                    // insertion will fail if there is already a key of print_z, so no need to check
                    bounds.entry(MapKeyF64(print_z)).or_insert(height);
                    bounds.entry(MapKeyF64(print_z - height)).or_insert(0.0); // the bottom_z of the layer
                }
            }
            let bounds_vec: Vec<(CoordF, CoordF)> = bounds.iter().map(|(k, v)| (k.0, *v)).collect();
            let mut it1 = 0usize;
            for it2 in 1..bounds_vec.len() {
                let z2 = bounds_vec[it2].0;
                let z1 = bounds_vec[it1].0;
                let dist = z2 - z1;
                if dist < min_layer_height - EPSILON {
                    continue;
                }
                // Insert intermediate layers.
                let n_layers_extra = (dist / max_layer_height).ceil() as usize;
                let step = dist / n_layers_extra as CoordF;
                let mut print_z = z1 + step;
                for _i in 0..n_layers_extra {
                    z_heights.insert(MapKeyF64(print_z), step);
                    print_z += step;
                }
                it1 = it2;
            }

            // map z_heights to layer_heights
            let mut obj_layer_nr: usize = 0;
            layer_heights.resize(z_heights.len(), LayerHeightData::default());
            for (i, (k, v)) in z_heights.iter().enumerate() {
                let print_z = k.0;
                let height = *v;
                while obj_layer_nr < obj_layer_zs.len() && (obj_layer_zs[obj_layer_nr] as f64) < print_z - height / 2.0 {
                    obj_layer_nr += 1;
                }
                layer_heights[i] = LayerHeightData::from_values(print_z, height, obj_layer_nr);
            }
        }

        // add support layers according to layer_heights
        let mut support_layer_nr: i32 = self.m_raft_layers as i32;
        for i in 0..layer_heights.len() {
            let (id, idx) = {
                let ts_layer = self.m_object.add_tree_support_layer(
                    support_layer_nr,
                    layer_heights[i].height,
                    layer_heights[i].print_z,
                    layer_heights[i].print_z - layer_heights[i].height / 2.0,
                );
                (ts_layer.layer.id(), 0usize)
            };
            let _ = idx;
            if id > self.m_raft_layers {
                // SupportLayer *lower_layer = m_object->get_support_layer(ts_layer->id() - 1);
                // The links are the positions in `m_support_layers`, which is what the ids of these layers are.
                let n = self.m_object.m_support_layers.len();
                if id - 1 < n {
                    self.m_object.m_support_layers[id - 1].layer.upper_layer_id = Some(n - 1);
                    self.m_object.m_support_layers[n - 1].layer.lower_layer_id = Some(id - 1);
                }
            }
            support_layer_nr += 1;
        }

        // re-distribute contact_nodes to support layers
        let mut contact_nodes2: Vec<Vec<NodeId>> = vec![Vec::new(); support_layer_nr as usize];
        for layer_nr in 0..self.contact_nodes.len() {
            if self.contact_nodes[layer_nr].is_empty() {
                continue;
            }
            let node1_print_z = self.m_ts_data.contact_nodes[self.contact_nodes[layer_nr][0]].print_z;
            // std::min_element: the first of the smallest
            let mut it = 0usize;
            for l in 1..layer_heights.len() {
                if (layer_heights[l].print_z - node1_print_z).abs() < (layer_heights[it].print_z - node1_print_z).abs() {
                    it = l;
                }
            }
            let layer_nr2 = it;
            let src = self.contact_nodes[layer_nr].clone();
            contact_nodes2[layer_nr2].extend(src);
        }
        self.contact_nodes = contact_nodes2;

        // adjust contact nodes' distance_to_top and support_roof_layers_below according to layer_heights
        // In case of very large top z distance, one gap layer is not enough, we need to split it into multiple layers
        for layer_nr in 0..self.contact_nodes.len() {
            if self.contact_nodes[layer_nr].is_empty() {
                continue;
            }
            let node1_height = self.m_ts_data.contact_nodes[self.contact_nodes[layer_nr][0]].height;
            let new_height = layer_heights[layer_nr].height;
            if (node1_height - new_height).abs() < EPSILON {
                continue;
            }
            if (self.top_z_distance as f64) < EPSILON && node1_height < EPSILON {
                continue; // top_z_distance==0, this is soluable interface
            }
            let mut accum_height: CoordF = 0.0;
            let mut num_layers: i32 = 0;
            let mut i = layer_nr as i32;
            while i >= 0 {
                if layer_heights[i as usize].height > EPSILON {
                    accum_height += layer_heights[i as usize].height;
                    num_layers += 1;
                    if accum_height > node1_height - EPSILON {
                        break;
                    }
                }
                i -= 1;
            }
            for &node in &self.contact_nodes[layer_nr] {
                let n = &mut self.m_ts_data.contact_nodes[node];
                n.height = new_height;
                n.distance_to_top = -num_layers;
                n.support_roof_layers_below += num_layers - 1;
            }
        }
        layer_heights
    }

    /// TreeSupport.cpp:6693-6705 `void TreeSupport::insert_dropped_node(std::vector<SupportNode*>& nodes_layer, SupportNode* p_node)`
    ///
    /// `std::find` over the pointers: the conflicting node is the node itself when it is in the layer already.
    pub fn insert_dropped_node(&mut self, nodes_layer: &mut Vec<NodeId>, p_node: NodeId) {
        let conflicting_node_it = nodes_layer.iter().position(|&n| n == p_node);
        let Some(pos) = conflicting_node_it else {
            //No conflict.
            nodes_layer.push(p_node);
            return;
        };
        let conflicting_node = nodes_layer[pos];
        let (dtt, roof) = {
            let p = &self.m_ts_data.contact_nodes[p_node];
            (p.distance_to_top, p.support_roof_layers_below)
        };
        let c = &mut self.m_ts_data.contact_nodes[conflicting_node];
        c.distance_to_top = c.distance_to_top.max(dtt);
        c.support_roof_layers_below = c.support_roof_layers_below.max(roof);
    }
}

// `std::clamp(v, lo, hi)`: `(v < lo) ? lo : (hi < v) ? hi : v`
#[inline]
fn std_clamp(v: f64, lo: f64, hi: f64) -> f64 {
    if v < lo {
        lo
    } else if hi < v {
        hi
    } else {
        v
    }
}

// ============================================================================
// libnest2d `placers::EdgeCache<ExPolygon>` (libnest2d/placers/nfpplacer.hpp:155-345), what
// generate_contact_points() uses of it: the cache of the edges, `circumference()` and `coords()`.
// ============================================================================

// nfpplacer.hpp:160-165 `struct ContourCache`
#[derive(Debug, Clone, Default)]
struct EdgeCacheContour {
    emap: Vec<(Point, Point)>,
    distances: Vec<f64>,
    full_distance: f64,
}

/// libnest2d `EdgeCache<ExPolygon>`
#[derive(Debug, Clone, Default)]
pub struct EdgeCache {
    contour_: EdgeCacheContour,
    holes_: Vec<EdgeCacheContour>,
}

impl EdgeCache {
    // nfpplacer.hpp:171-174 `static double length(const Edge &e)`: sqrt of `_Segment::sqlength<double>()`
    // (geometry_traits.hpp:334-339, `a * a + b * b`).
    fn length(e: &(Point, Point)) -> f64 {
        let a = e.1.x as f64 - e.0.x as f64;
        let b = e.1.y as f64 - e.0.y as f64;
        a.mul_add(a, b * b).sqrt()
    }

    // nfpplacer.hpp:177-194, 197-213: the edges of one path, closed when its ends differ.
    fn create_contour_cache(points: &[Point]) -> EdgeCacheContour {
        let mut cache = EdgeCacheContour::default();
        if points.is_empty() {
            return cache;
        }
        cache.distances.reserve(points.len());
        for i in 0..points.len() - 1 {
            cache.emap.push((points[i], points[i + 1]));
            cache.full_distance += Self::length(&cache.emap[cache.emap.len() - 1]);
            cache.distances.push(cache.full_distance);
        }
        if points[0] != points[points.len() - 1] {
            cache.emap.push((points[points.len() - 1], points[0]));
            cache.full_distance += Self::length(&cache.emap[cache.emap.len() - 1]);
            cache.distances.push(cache.full_distance);
        }
        cache
    }

    /// nfpplacer.hpp:303-306 `EdgeCache(const RawShape& sh)` → `createCache(sh)` (176-217)
    pub fn new(sh: &ExPolygon) -> Self {
        let mut cache = EdgeCache { contour_: Self::create_contour_cache(&sh.contour.points), holes_: Vec::new() };
        for h in &sh.holes {
            cache.holes_.push(Self::create_contour_cache(&h.points));
        }
        cache
    }

    // nfpplacer.hpp:258-289 `Vertex coords(const ContourCache& cache, double distance) const`
    fn coords_cache(cache: &EdgeCacheContour, mut distance: f64) -> Point {
        if cache.distances.is_empty() || cache.emap.is_empty() {
            return Point::new(0, 0);
        }
        // Zero-perimeter contour (all edges collapsed to one point)
        if cache.full_distance <= 0.0 {
            return cache.emap[0].0;
        }
        if distance > 1.0 {
            distance %= 1.0;
        }
        // distance is from 0.0 to 1.0, we scale it up to the full length of the circumference
        let d = distance * cache.full_distance;
        let distances = &cache.distances;
        // Magic: we find the right edge in log time (std::lower_bound)
        let idx = distances.partition_point(|&x| x < d);
        let edge = cache.emap[idx]; // extrac the edge
        // Get the remaining distance on the target edge
        let ed = d - (if idx > 0 { distances[idx - 1] } else { 0.0 });
        // geometry_traits.hpp:455-468 `_Segment::angleToXaxis()`
        let dx = edge.1.x - edge.0.x;
        let dy = edge.1.y - edge.0.y;
        let mut angle = (dy as f64).atan2(dx as f64);
        if angle.is_sign_negative() {
            angle += 2.0 * std::f64::consts::PI;
        }
        // Get the point on the edge which lies in ed distance from the start
        Point::new(edge.0.x + (ed * angle.cos()).round() as Coord, edge.0.y + (ed * angle.sin()).round() as Coord)
    }

    /// nfpplacer.hpp:319-321
    pub fn coords(&self, distance: f64) -> Point {
        Self::coords_cache(&self.contour_, distance)
    }
    /// nfpplacer.hpp:323-326
    pub fn coords_hole(&self, hidx: usize, distance: f64) -> Point {
        Self::coords_cache(&self.holes_[hidx], distance)
    }
    /// nfpplacer.hpp:328-330
    pub fn circumference(&self) -> f64 {
        self.contour_.full_distance
    }
    /// nfpplacer.hpp:332-334
    pub fn circumference_hole(&self, hidx: usize) -> f64 {
        self.holes_[hidx].full_distance
    }
    /// `holeCount()`
    pub fn hole_count(&self) -> usize {
        self.holes_.len()
    }
}

// ClipperUtils.cpp:683-686 `ExPolygons diff_clipped(const ExPolygons &subject, const ExPolygons &clip, ApplySafetyOffset::No)`
fn diff_clipped(subject: &[ExPolygon], clip: &[ExPolygon]) -> ExPolygons {
    let mut bbox = crate::geometry::get_extents(subject);
    // .inflated(SCALED_EPSILON)
    let eps = crate::libslic3r::SCALED_EPSILON as Coord;
    bbox.min = Point::new(bbox.min.x - eps, bbox.min.y - eps);
    bbox.max = Point::new(bbox.max.x + eps, bbox.max.y + eps);
    let clipped = crate::clipper_utils::clip_clipper_polygons_with_subject_bbox_expolygons(&clip.to_vec(), &bbox, false);
    diff_ex_polygons(subject, &clipped)
}

// ClipperUtils.hpp `Lines intersection_ln(const Line &subject, const Polygons &clip)` = `_clipper_ln(ctIntersection, {subject}, clip)`
// (ClipperUtils.cpp): the line as a polyline clipped by the polygons, each piece as a line of its ends.
fn intersection_ln(subject: &Line, clip: &[Polygon]) -> Lines {
    let polylines = vec![crate::geometry::Polyline::from_points(vec![subject.a, subject.b])];
    let polylines = crate::clipper_utils::intersection_pl_clib(&polylines, &crate::clipper_utils::polygons_as_paths(clip));
    let mut retval: Lines = Vec::new();
    for polyline in &polylines {
        if polyline.points.len() >= 2 {
            retval.push(Line::new(polyline.points[0], polyline.points[polyline.points.len() - 1]));
        }
    }
    retval
}

// ClipperUtils.hpp:392-393 `ExPolygons shrink_ex(const ExPolygons &polygons, const float delta)` = offset_ex(polygons, -delta)
fn shrink_ex(expolygons: &[ExPolygon], delta: f32) -> ExPolygons {
    offset_ex(expolygons, -delta)
}

// SupportCommon.cpp:3037-3051 `ExPolygons safe_union(const ExPolygons first, const ExPolygons second = {})`.
// A local copy: `safe_union_ex` of support_common.rs unions on another clipper.
fn safe_union(first: &[ExPolygon], second: &[ExPolygon]) -> ExPolygons {
    use crate::clipper::{EndType, JoinType};
    use crate::support::support_common::{cu, cu_expolygon};
    let mut result: ExPolygons = Vec::new();
    if !first.is_empty() || !second.is_empty() {
        result = cu_expolygon::union_ex_2(first, second);
        if result.is_empty() {
            // just take the few lines we have, and offset them a tiny bit. Needs to be offsetPolylines, as offset may aleady have problems with the area.
            let to_polylines = |src: &[ExPolygon]| -> crate::geometry::Polylines {
                let mut out = Vec::new();
                for expoly in src {
                    out.extend(cu_expolygon::to_polylines_closed(expoly));
                }
                out
            };
            // scaled<float>(0.002)
            let delta = (0.002 / SCALING_FACTOR) as f32;
            let offset = |src: &[ExPolygon]| -> Polygons {
                let polylines = to_polylines(src);
                // raw_offset() of no paths is no paths
                if polylines.is_empty() {
                    Vec::new()
                } else {
                    cu::offset_polylines(&polylines, delta, JoinType::jtMiter, 1.2, EndType::etOpenButt)
                }
            };
            let result_polys = cu::union_2(&offset(first), &offset(second));
            for poly in result_polys {
                result.push(ExPolygon::new(poly));
            }
        }
    }
    result
}

// SupportCommon.hpp:218-256 `template<typename CollisionPolyType> ExPolygons safe_offset_inc(const ExPolygons &me,
// coord_t distance, const CollisionPolyType &collision, coord_t safe_step_size, coord_t last_step_offset_without_check,
// size_t min_amount_offset)` with CollisionPolyType = ExPolygons. A local copy: `safe_offset_inc_ex` of
// support_common.rs takes the collision as Polygons and runs on another clipper.
fn safe_offset_inc(
    me: &ExPolygons,
    distance: Coord,
    collision: &[ExPolygon],
    safe_step_size: Coord,
    last_step_offset_without_check: Coord,
    min_amount_offset: usize,
) -> ExPolygons {
    use crate::clipper::JoinType;
    use crate::support::support_common::cu;
    let mut do_final_difference = last_step_offset_without_check == 0;
    let mut ret: ExPolygons = safe_union(me, &[]); // ensure sane input

    // Trim the collision polygons with the region of interest for diff() efficiency.
    let mut collision_trimmed_buffer: Polygons = Vec::new();
    macro_rules! collision_trimmed {
        () => {{
            if collision_trimmed_buffer.is_empty() && !collision.is_empty() {
                // get_extents(ret).inflated(std::max(0, distance) + SCALED_EPSILON): the sum is a double,
                // truncated to the coord_t inflated() takes
                let delta = (0.max(distance) as f64 + scale_(EPSILON)) as Coord;
                let mut bbox = crate::geometry::get_extents(&ret);
                bbox.min = Point::new(bbox.min.x - delta, bbox.min.y - delta);
                bbox.max = Point::new(bbox.max.x + delta, bbox.max.y + delta);
                collision_trimmed_buffer =
                    crate::clipper_utils::clip_clipper_polygons_with_subject_bbox_expolygons(&collision.to_vec(), &bbox, false);
            }
            &collision_trimmed_buffer
        }};
    }
    // scaled<float>(0.01), the arc tolerance of jtRound
    let arc_tolerance = (0.01 / SCALING_FACTOR) as f32 as f64;
    // scaled<double>(0.015)
    let simplify_tolerance = 0.015 / SCALING_FACTOR;

    if distance == 0 {
        return if do_final_difference {
            cu::diff_ex_ep(&ret, collision_trimmed!(), cu::ApplySafetyOffset::No)
        } else {
            cu::union_ex_expolygons(&ret)
        };
    }
    if safe_step_size < 0 || last_step_offset_without_check < 0 {
        // BOOST_LOG_TRIVIAL(error) << "Offset increase got invalid parameter!";
        return if do_final_difference {
            cu::diff_ex_ep(&ret, collision_trimmed!(), cu::ApplySafetyOffset::No)
        } else {
            cu::union_ex_expolygons(&ret)
        };
    }

    // coord_t is 32 bits (libslic3r.h:40), so is the arithmetic of the steps
    let distance = distance as i32;
    let last_step_offset_without_check = last_step_offset_without_check as i32;
    let safe_step_size = safe_step_size as i32;
    let mut step_size: i32 = safe_step_size;
    let mut steps: i32 = if distance > last_step_offset_without_check { (distance - last_step_offset_without_check) / step_size } else { 0 };
    if distance - steps * step_size > last_step_offset_without_check {
        if (steps + 1) * step_size <= distance {
            // This will be the case when last_step_offset_without_check >= safe_step_size
            steps += 1;
        } else {
            do_final_difference = true;
        }
    }
    if steps + ((distance < last_step_offset_without_check || (distance % step_size) != 0) as i32) < min_amount_offset as i32 && min_amount_offset > 1 {
        // yes one can add a bool as the standard specifies that a result from compare operators has to be 0 or 1
        // reduce the stepsize to ensure it is offset the required amount of times
        // `distance / min_amount_offset`: the division is unsigned (size_t)
        step_size = ((distance as i64 as u64) / min_amount_offset as u64) as i32;
        if step_size >= safe_step_size {
            // effectivly reduce last_step_offset_without_check
            step_size = safe_step_size;
            steps = min_amount_offset as i32;
        } else {
            steps = distance / step_size;
        }
    }
    // offset in steps
    for i in 0..steps {
        ret = cu::diff_ex_ep(
            &cu::offset_ex_expolygons(&ret, step_size as f32, JoinType::jtRound, arc_tolerance),
            collision_trimmed!(),
            cu::ApplySafetyOffset::No,
        );
        // ensure that if many offsets are done the performance does not suffer extremely by the new vertices of jtRound.
        if i % 10 == 7 {
            ret = expolygons_simplify(&ret, simplify_tolerance);
        }
    }
    // offset the remainder
    let last_offset: f32 = (distance - steps * step_size) as f32;
    if last_offset as f64 > scale_(EPSILON) {
        ret = cu::offset_ex_expolygons(&ret, (distance - steps * step_size) as f32, JoinType::jtRound, arc_tolerance);
    }
    ret = expolygons_simplify(&ret, simplify_tolerance);

    if do_final_difference {
        ret = cu::diff_ex_ep(&ret, collision_trimmed!(), cu::ApplySafetyOffset::No);
    }
    cu::union_ex_expolygons(&ret)
}

// ClipperUtils.cpp:567-569 `ExPolygons offset_ex(const ExPolygon &expolygon, const float delta, ...)`, the overload
// `offset_ex({expolygon}, delta)` and `offset_ex(expolygons[0], delta)` resolve to:
// `ClipperPaths_to_Slic3rExPolygons(expolygon_offset(...))`, an even-odd union, not the PolyTree of
// `offset_ex(const ExPolygons &, ...)`.
fn offset_ex_expolygon(expolygon: &ExPolygon, delta: f32) -> ExPolygons {
    crate::support::support_common::cu_expolygon::offset_ex(expolygon, delta, crate::clipper::JoinType::jtMiter, 3.0)
}

/// The locals the lambdas of `TreeSupport::generate_contact_points()` capture, per layer (TreeSupport.cpp:6470-6514).
struct ContactPointsLayer {
    layer_nr: usize,
    curr_nodes: Vec<NodeId>,
    already_inserted: HashSet<Point>,
    bottom_z: CoordF,
    added: bool,
    is_sharp_tail: bool,
    overhang_top_z: CoordF,
    overhang_gap_layers: i32,
    overhang_gap_height: CoordF,
}

impl<'a> TreeSupport<'a> {
    // TreeSupport.cpp:6479-6494 the lambda `set_overhang_top_z`
    fn contact_points_set_overhang_top_z(&self, ctx: &mut ContactPointsLayer, top_z: CoordF, gap_layers: i32, presplit_gap: bool) {
        ctx.overhang_top_z = top_z;
        ctx.overhang_gap_layers = gap_layers;
        ctx.overhang_gap_height = top_z;
        if !presplit_gap {
            return;
        }
        let mut accum_height: CoordF = 0.0;
        ctx.overhang_gap_layers = 0;
        let mut i = ctx.layer_nr as i32 - 1;
        while i >= 0 {
            accum_height += self.m_object.get_layer(i).height;
            ctx.overhang_gap_layers += 1;
            if accum_height > top_z - EPSILON {
                break;
            }
            i -= 1;
        }
        ctx.overhang_gap_height = self.m_object.get_layer(ctx.layer_nr as i32 - 1).height;
    }

    // TreeSupport.cpp:6495-6516 the lambda `insert_point`
    #[allow(clippy::too_many_arguments)]
    fn contact_points_insert_point(
        &mut self,
        ctx: &mut ContactPointsLayer,
        pt: Point,
        overhang: &ExPolygon,
        radius: f64,
        force_add: bool,
        add_interface: bool,
        radius_scaled: Coord,
        support_roof_layers: usize,
    ) -> Option<NodeId> {
        let hash_pos = point_div(pt, (radius_scaled + 1) / 1);
        let mut contact_node: Option<NodeId> = None;
        if force_add || !ctx.already_inserted.contains(&hash_pos) {
            ctx.already_inserted.insert(hash_pos);
            let to_buildplate = true;
            let roof_layers: usize = if add_interface { support_roof_layers } else { 0 };
            // add a new node as a virtual node which acts as the invisible gap between support and object
            // distance_to_top<0: it's virtual (-1, or minus the gap layers once pre-split)
            // print_z=object_layer->bottom_z: it directly contacts the bottom
            // height=z_distance_top: it's height is exactly the gap distance (the layer height once pre-split)
            // dist_mm_to_top=0: it directly contacts the bottom
            if ctx.bottom_z - ctx.overhang_top_z < self.m_object.get_layer(0).print_z {
                return contact_node; // dont add it if overlapping with the initial layer
            }
            let node = self.m_ts_data.create_node(
                pt,
                -ctx.overhang_gap_layers,
                ctx.layer_nr as i32 - 1,
                (roof_layers as i64 + ctx.overhang_gap_layers as i64) as i32,
                to_buildplate,
                SupportNode::NO_PARENT,
                ctx.bottom_z,
                ctx.overhang_gap_height,
                0.0,
                radius,
            );
            let n = &mut self.m_ts_data.contact_nodes[node];
            n.overhang = overhang.clone();
            n.is_sharp_tail = ctx.is_sharp_tail;
            n.carried_mass = (M_PI * sq(radius.max(self.tip_radius))) as f32;
            ctx.curr_nodes.push(node);
            ctx.added = true;
            contact_node = Some(node);
        }
        contact_node
    }

    /// TreeSupport.cpp:6389-6691 `void TreeSupport::generate_contact_points()`
    ///
    /// FIDELITY-NOTE: the tbb::parallel_for over the layers runs in order; every layer fills its own
    /// `contact_nodes[layer_nr - 1]`, and `all_nodes` (a concurrent vector whose order the threads decide) is
    /// filled in layer order. `layer->lower_layer` is the object layer below.
    pub fn generate_contact_points(&mut self) {
        let config = self.m_object.config();
        let point_spread: CoordF = scale_(cfg_float(config, "tree_support_branch_distance"));
        let max_bridge_length: CoordF = scale_(cfg_float(config, "max_bridge_length"));
        let radius_scaled: Coord = scale_(self.base_radius) as Coord;
        let _on_buildplate_only = cfg_bool(config, "support_on_build_plate_only");
        let roof_enabled = cfg_int(config, "support_interface_top_layers") > 0;
        let force_tip_to_roof = roof_enabled && self.m_support_params.soluble_interface;
        let extrusion_width: CoordF = crate::flow::Flow::extrusion_width_for_nozzle(
            &cfg_float_option(config, "line_width"),
            crate::print_config::config_def::primary_nozzle_diameter(self.m_print.full_print_config()) as f32,
            support_nozzle_diameter(self.m_print, cfg_int(config, "support_filament") as u32),
            crate::print_config::config_def::multi_nozzle_diameter_mode_enabled(self.m_print.full_print_config()),
        );
        //First generate grid points to cover the entire area of the print.
        let bounding_box = self.m_object.bounding_box();
        let bounding_box_size = Point::new(bounding_box.max.x - bounding_box.min.x, bounding_box.max.y - bounding_box.min.y);
        let rotate_angle: f64 = 22.0 / 180.0 * M_PI;
        let center = point_div(Point::new(bounding_box.max.x + bounding_box.min.x, bounding_box.max.y + bounding_box.min.y), 2);
        let sin_angle = rotate_angle.sin();
        let cos_angle = rotate_angle.cos();
        let rotated_dims = point_div(
            point_from_f64(
                (bounding_box_size.x as f64).mul_add(cos_angle, bounding_box_size.y as f64 * sin_angle),
                (bounding_box_size.x as f64).mul_add(sin_angle, bounding_box_size.y as f64 * cos_angle),
            ),
            2,
        );

        let mut grid_points: Vec<Point> = Vec::new();
        let sample_step: CoordF = point_spread.max(max_bridge_length / 2.0);
        // `auto x = -rotated_dims(0)` is a coord_t: `x += sample_step` truncates every step.
        let mut x: Coord = -rotated_dims.x;
        while x < rotated_dims.x {
            let mut y: Coord = -rotated_dims.y;
            while y < rotated_dims.y {
                let mut pt = Point::new(x, y);
                pt = point_rotate(pt, cos_angle, sin_angle);
                pt = Point::new(pt.x + center.x, pt.y + center.y);
                if bounding_box.contains_point(&pt) {
                    grid_points.push(pt);
                }
                y = (y as f64 + sample_step) as Coord;
            }
            x = (x as f64 + sample_step) as Coord;
        }

        let layer_height: CoordF = cfg_float(config, "layer_height");
        let z_distance_top: CoordF = self.top_z_distance as f64;
        // const int z_distance_top_layers = round_up_divide(scale_(m_top_z_max), scale_(layer_height)) + 1;
        // round_up_divide<double>: (dividend + divisor - 1) / divisor
        let z_distance_top_layers: i32 = {
            let dividend = scale_(self.m_top_z_max as f64);
            let divisor = scale_(layer_height);
            ((dividend + divisor - 1.0) / divisor + 1.0) as i32
        }; //Support must always be 1 layer below overhang.
        let gap_layers: i32 = if z_distance_top == 0.0 { 0 } else { 1 };
        // BBS multi-diameter: the gap follows the overhang's nozzle, so one layer's contacts may need different
        // gaps. plan_layer_heights() splits a gap into virtual layers from the first contact of each layer only,
        // so split each contact here instead, exactly as it would.
        let presplit_gap = !self.m_top_z_outlines.is_empty();

        let mut support_roof_layers: usize = cfg_int(config, "support_interface_top_layers") as usize;
        if support_roof_layers > 0 {
            support_roof_layers += 1; // BBS: add a normal support layer below interface (if we have interface)
        }
        let support_threshold_angle = cfg_int(config, "support_threshold_angle");
        let _thresh_angle: CoordF =
            (89.0_f32).min(if (support_threshold_angle as f64) < EPSILON { 30.0_f32 } else { support_threshold_angle as f32 }) as f64;
        let support_expansion = cfg_float(config, "support_expansion");
        let support_object_xy_distance = cfg_float(config, "support_object_xy_distance");
        let mut xy_expansion: CoordF = scale_(support_expansion);
        if self.m_support_params.soluble_interface && xy_expansion < EPSILON {
            xy_expansion = scale_(2.0);
        }

        // fix bug of generating support for very thin objects
        let n_layers = self.m_object.layers().len();
        if n_layers <= (z_distance_top_layers + 1) as usize {
            return;
        }

        self.contact_nodes.clear();
        self.contact_nodes.resize(n_layers, Vec::new());

        // add vertical enforcer points
        // Layer.hpp:411-418 zs_from_layers(): `(float)l->slice_z`
        let zs: Vec<f32> = self.m_object.layers().iter().map(|l| l.slice_z as f32).collect();
        let mut vertical_enforcer_points_by_layers: Vec<Vec<(Point, Point)>> = vec![Vec::new(); self.m_object.layer_count()];
        for pt_and_normal in &self.m_vertical_enforcer_points {
            let pt = pt_and_normal.0;
            let normal = pt_and_normal.1; // normal seems useless
            let iter = zs.partition_point(|&z| z < pt[2]);
            if iter != zs.len() {
                let layer_nr = iter;
                if layer_nr > 0 && layer_nr < self.contact_nodes.len() {
                    // { to_2d(pt).cast<coord_t>(), scaled(to_2d(normal)) }: Point.hpp:550-554 `(v / SCALING_FACTOR).cast<coord_t>()`
                    // on a float vector, the divisor as a float
                    vertical_enforcer_points_by_layers[layer_nr].push((
                        Point::new(pt[0] as Coord, pt[1] as Coord),
                        Point::new((normal[0] / SCALING_FACTOR as f32) as Coord, (normal[1] / SCALING_FACTOR as f32) as Coord),
                    ));
                }
            }
        }

        let mut nonempty_layers: i32 = 0;
        let mut all_nodes: Vec<[f32; 3]> = Vec::new();
        let soluble_interface = self.m_support_params.soluble_interface;
        let support_style = self.m_support_params.support_style;
        let xy_distance = self.m_ts_data.m_xy_distance;
        for layer_nr in 1..n_layers {
            if (self.canceled)() {
                break;
            }
            let (loverhangs_with_type, layer_bottom_z, lower_lslices_extrudable) = {
                let layer = self.m_object.get_layer(layer_nr as i32);
                let lower = self.m_object.get_layer(layer_nr as i32 - 1);
                (layer.loverhangs_with_type.clone(), layer.bottom_z(), lower.lslices_extrudable.clone())
            };
            let mut ctx = ContactPointsLayer {
                layer_nr,
                curr_nodes: std::mem::take(&mut self.contact_nodes[layer_nr - 1]),
                already_inserted: HashSet::new(),
                bottom_z: layer_bottom_z,
                added: false, // Did we add a point this way?
                is_sharp_tail: false,
                overhang_top_z: z_distance_top,
                overhang_gap_layers: gap_layers,
                overhang_gap_height: z_distance_top,
            };

            // auto extrudable_collision = offset_ex(layer->lower_layer->lslices_extrudable, m_ts_data->m_xy_distance);
            let extrudable_collision = offset_ex(&lower_lslices_extrudable, xy_distance as f32);
            for overhang_with_type in &loverhangs_with_type {
                let overhang_part = &overhang_with_type.0;
                let overhang_type = overhang_with_type.1;
                ctx.is_sharp_tail = (overhang_type & OverhangType::SHARP_TAIL.0 as i32) != 0;
                let top_z = self.top_z_for_overhang(layer_nr, overhang_part) as CoordF;
                self.contact_points_set_overhang_top_z(&mut ctx, top_z, gap_layers, presplit_gap);
                let mut add_interface = force_tip_to_roof || overhang_part.area() > self.minimum_roof_area;
                let relevant_forbidden = self.get_collision(0.0, layer_nr - 1);
                let mut overhangs: ExPolygons = vec![overhang_part.clone()];
                let mut overhangs_regular: ExPolygons;
                if !ctx.is_sharp_tail && (support_expansion > EPSILON) {
                    overhangs = safe_offset_inc(
                        &vec![overhang_part.clone()],
                        scale_(support_expansion) as Coord,
                        &relevant_forbidden,
                        scale_(TS_MIN_BRANCH_RADIUS * 1.75) as Coord,
                        0,
                        1,
                    );
                }
                if support_style == SupportMaterialStyle::SmsTreeHybrid as i32
                    && (overhang_type & (OverhangType::BIG_FLAT.0 as i32 | OverhangType::THIN_PLATE.0 as i32)) != 0
                {
                    let outlines_below_offset =
                        offset_ex(&self.m_ts_data.m_layer_outlines_below[layer_nr - 1], scale_(support_object_xy_distance) as f32);
                    overhangs_regular = intersection_ex(&overhangs, &outlines_below_offset);
                    let mut overhangs_normal = offset2_ex(
                        &diff_ex(std::slice::from_ref(overhang_part), &overhangs_regular),
                        scale_(extrusion_width) as f32,
                        -scale_(extrusion_width) as f32,
                    );
                    if add_interface && xy_expansion > EPSILON && !ctx.is_sharp_tail {
                        overhangs_normal = safe_offset_inc(
                            &overhangs_normal,
                            xy_expansion as Coord,
                            &outlines_below_offset,
                            scale_(TS_MIN_BRANCH_RADIUS * 1.75) as Coord,
                            0,
                            1,
                        );
                    }
                    // if the outside area is still big, we can need normal nodes
                    let gap_width: Coord = (scale_(extrusion_width / 2.0) + scale_(xy_distance)) as Coord;
                    let mut overhangs_normal_split: ExPolygons = Vec::new();
                    let collision_layer = self.get_collision(0.0, layer_nr);
                    for overhang in &overhangs_normal {
                        let sub_overhangs = overhang.split_expoly_with_holes(gap_width, &collision_layer);
                        if !sub_overhangs.is_empty() {
                            for sub in sub_overhangs {
                                overhangs_normal_split.push(sub);
                            }
                        } else {
                            overhangs_normal_split.push(overhang.clone());
                        }
                    }
                    for overhang in &overhangs_normal_split {
                        if !is_stable(layer_bottom_z as f32, overhang, 0.0) || overhang.area() < sq(scale_(2.0)) {
                            let unstable_overhangs = intersection_ex(std::slice::from_ref(overhang), &overhangs);
                            overhangs_regular.extend(unstable_overhangs);
                            continue;
                        }
                        let overhang_bounds = get_extents_expoly(overhang);
                        let radius = bbox_radius(&overhang_bounds) * SCALING_FACTOR;
                        let candidate =
                            point_div(Point::new(overhang_bounds.min.x + overhang_bounds.max.x, overhang_bounds.min.y + overhang_bounds.max.y), 2);
                        let Some(contact_node) =
                            self.contact_points_insert_point(&mut ctx, candidate, overhang, radius, true, true, radius_scaled, support_roof_layers)
                        else {
                            continue;
                        };
                        let n = &mut self.m_ts_data.contact_nodes[contact_node];
                        n.type_ = TreeNodeType::EPolygon;
                        n.carried_mass = n.carried_mass.max((overhang.area() * SCALING_FACTOR * SCALING_FACTOR) as f32);
                        ctx.curr_nodes.push(contact_node);
                    }
                } else {
                    overhangs_regular = overhangs.clone();
                }
                // if (!is_sharp_tail && layer->lower_layer)
                if !ctx.is_sharp_tail {
                    overhangs_regular = diff_ex(&overhangs_regular, &extrudable_collision);
                }
                for overhang in &overhangs_regular {
                    let nodes_before = ctx.curr_nodes.len();
                    if ctx.is_sharp_tail && !soluble_interface && overhang.area() < sq(scale_(2.0)) {
                        add_interface = false;
                    }
                    let overhang_bounds = get_extents_expoly(overhang);
                    let radius: f64 = std_clamp(bbox_radius(&overhang_bounds) * SCALING_FACTOR, self.tip_radius, self.base_radius);
                    // add supports at corners for both auto and manual overhangs, github #2008
                    let points = &overhang.contour.points;
                    let n_size = points.len() as i32;
                    for i in 0..n_size {
                        let pt = points[i as usize];
                        let v1 = vec2d_normalized(pt, points[((i - 1 + n_size) % n_size) as usize]);
                        let v2 = vec2d_normalized(pt, points[((i + 1) % n_size) as usize]);
                        if v1.0 * v2.0 + v1.1 * v2.1 > -0.7 {
                            // angle smaller than 135 degrees
                            let contact_node =
                                self.contact_points_insert_point(&mut ctx, pt, overhang, radius, false, add_interface, radius_scaled, support_roof_layers);
                            if let Some(contact_node) = contact_node {
                                self.m_ts_data.contact_nodes[contact_node].is_corner = true;
                            }
                        }
                    }
                    // add supports along contours
                    let edge_cache = EdgeCache::new(overhang);
                    for i in 0..edge_cache.hole_count() + 1 {
                        let step = point_spread / (if i == 0 { edge_cache.circumference() } else { edge_cache.circumference_hole(i - 1) });
                        let mut distance = 0.0_f64;
                        while distance < 1.0 {
                            let pt = if i == 0 { edge_cache.coords(distance) } else { edge_cache.coords_hole(i - 1, distance) };
                            self.contact_points_insert_point(&mut ctx, pt, overhang, radius, false, add_interface, radius_scaled, support_roof_layers);
                            distance += step;
                        }
                    }
                    // don't add inner supports for sharp tails
                    if ctx.is_sharp_tail {
                        let mut cent = polygon_centroid(&overhang.contour);
                        // For ring/concave sharp tails the centroid may fall outside the overhang
                        // (e.g. inside its hole). In that case project it onto the nearest edge of
                        // the overhang so the sharp tail still gets a contact point.
                        if !is_inside_ex(&vec![overhang.clone()], &cent) {
                            cent = overhang.point_projection(&cent);
                        }
                        self.contact_points_insert_point(&mut ctx, cent, overhang, radius, false, add_interface, radius_scaled, support_roof_layers);
                    } else {
                        // add inner supports
                        // overhang_bounds.inflated(-radius_scaled);  (the inflated copy is dropped)
                        let overhang_inner = offset_ex(std::slice::from_ref(overhang), -(radius_scaled as f32));
                        for &candidate in &grid_points {
                            if overhang_bounds.contains_point(&candidate) {
                                // BBS: move_inside_expoly shouldn't be used if candidate is already inside, as it moves point to boundary and the inside is not well supported!
                                let is_inside = is_inside_ex(&overhang_inner, &candidate);
                                if is_inside {
                                    self.contact_points_insert_point(
                                        &mut ctx,
                                        candidate,
                                        overhang,
                                        radius,
                                        false,
                                        add_interface,
                                        radius_scaled,
                                        support_roof_layers,
                                    );
                                }
                            }
                        }
                    }
                    let nodes_added = ctx.curr_nodes.len() - nodes_before;
                    if nodes_added > 0 {
                        let overhang_area_mm2 = (overhang.area() * SCALING_FACTOR * SCALING_FACTOR) as f32;
                        let node_mass = (overhang_area_mm2 / nodes_added as f32).max((M_PI * sq(self.tip_radius)) as f32);
                        for idx in nodes_before..ctx.curr_nodes.len() {
                            self.m_ts_data.contact_nodes[ctx.curr_nodes[idx]].carried_mass = node_mass;
                        }
                    }
                }
            }
            if !vertical_enforcer_points_by_layers[layer_nr].is_empty() {
                let top_z = self.top_z_at(layer_nr) as CoordF;
                self.contact_points_set_overhang_top_z(&mut ctx, top_z, gap_layers, presplit_gap);
            }
            for pt_and_normal in &vertical_enforcer_points_by_layers[layer_nr] {
                ctx.is_sharp_tail = true; // fake it as sharp tail point so the contact distance will be 0
                let vertical_enforcer_point = pt_and_normal.0;
                // insert_point(vertical_enforcer_point, ExPolygon(), false): the radius is `false`, 0
                let node = self.contact_points_insert_point(
                    &mut ctx,
                    vertical_enforcer_point,
                    &ExPolygon::default(),
                    0.0,
                    false,
                    true,
                    radius_scaled,
                    support_roof_layers,
                );
                if let Some(node) = node {
                    self.m_ts_data.contact_nodes[node].skin_direction = pt_and_normal.1;
                }
            }
            if !ctx.curr_nodes.is_empty() {
                nonempty_layers += 1;
            }
            for &node in &ctx.curr_nodes {
                let n = &self.m_ts_data.contact_nodes[node];
                all_nodes.push([n.position.x as f32, n.position.y as f32, scale_(n.print_z) as f32]);
            }
            self.contact_nodes[layer_nr - 1] = ctx.curr_nodes;
        }

        let n_nodes = all_nodes.len() as i32;
        self.avg_node_per_layer = 0;
        self.nodes_angle = 0.0;
        if n_nodes > 0 {
            self.avg_node_per_layer = n_nodes / nonempty_layers;
            // get orientation of nodes by line fitting
            // line: y=kx+b, where
            //       k=tan(nodes_angle)=(n\sum{xy}-\sum{x}\sum{y})/(n\sum{x^2}-\sum{x}^2)
            let (mut mx, mut my, mut mxy, mut mx2) = (0.0_f32, 0.0_f32, 0.0_f32, 0.0_f32);
            for pt in &all_nodes {
                let x: f32 = (pt[0] as f64 * SCALING_FACTOR) as f32;
                let y: f32 = (pt[1] as f64 * SCALING_FACTOR) as f32;
                mx += x;
                my += y;
                mxy = x.mul_add(y, mxy);
                mx2 = x.mul_add(x, mx2);
            }
            let n = n_nodes as f32;
            self.nodes_angle = n.mul_add(mxy, -(mx * my)).atan2(n.mul_add(mx2, -(mx * mx)));
        }
    }
}

// `(a - b).cast<double>().normalized()` of Eigen: the vector over its norm when the squared norm is positive.
fn vec2d_normalized(a: Point, b: Point) -> (f64, f64) {
    let x = (a.x - b.x) as f64;
    let y = (a.y - b.y) as f64;
    let z = x * x + y * y;
    if z > 0.0 {
        let n = z.sqrt();
        (x / n, y / n)
    } else {
        (x, y)
    }
}

// TreeSupport.cpp:5237-5255 the lambda `is_line_cut_by_contour` of drop_nodes(); the cache is the
// `tbb::concurrent_unordered_map<Line, bool, LineHash>` of the layer.
fn is_line_cut_by_contour(cache: &mut HashMap<(Point, Point), bool>, layer_contours: &Polygons, a: Point, b: Point) -> bool {
    if let Some(&cut) = cache.get(&(a, b)) {
        if cut {
            return true;
        }
    } else {
        profiler().tic();
        let ln = Line::new(b, a);
        let pls_intersect = intersection_ln(&ln, layer_contours);
        // insert() keeps an element already there
        cache.entry((a, b)).or_insert(!pls_intersect.is_empty());
        cache.entry((b, a)).or_insert(!pls_intersect.is_empty());
        profiler().stage_add(TreeSupportStage::StageIntersectionLn);
        if !pls_intersect.is_empty() {
            return true;
        }
    }
    false
}

impl<'a> TreeSupport<'a> {
    // TreeSupport.cpp:5165-5176 the lambda `get_max_move_dist` of drop_nodes()
    fn get_max_move_dist(&mut self, node: NodeId, tan_angle: f64, power: i32) -> f64 {
        if self.m_ts_data.contact_nodes[node].max_move_dist == 0.0 {
            let radius = self.get_radius(node);
            self.m_ts_data.contact_nodes[node].radius = radius;
            let (height, print_z) = (self.m_ts_data.contact_nodes[node].height, self.m_ts_data.contact_nodes[node].print_z);
            self.m_ts_data.contact_nodes[node].max_move_dist = (tan_angle * height).min(self.support_flare_step_width(print_z, height));
        }
        let mut move_dist = self.m_ts_data.contact_nodes[node].max_move_dist;
        if power == 2 {
            move_dist = sq(move_dist);
        }
        move_dist
    }

    /// TreeSupport.cpp:5147-6034 `void TreeSupport::drop_nodes()` (USE_TREE_SUPPORT_CARRIED_MASS 1)
    ///
    /// FIDELITY-NOTE: the second pass is a `tbb::parallel_for_each` over the nodes of a part, which appends
    /// the nodes of the layer below under a mutex in the order the threads reach it; here the nodes are taken
    /// in order, the schedule of one thread. The avoidance of all radii is precalculated in order as well.
    pub fn drop_nodes(&mut self) {
        macro_rules! n {
            ($id:expr) => {
                self.m_ts_data.contact_nodes[$id]
            };
        }
        let config = self.m_object.config();
        // Use Minimum Spanning Tree to connect the points on each layer and move them while dropping them down.
        let support_extrusion_width: CoordF = self.m_support_params.support_extrusion_width;
        let layer_height: CoordF = cfg_float(config, "layer_height");
        let angle: f64 = cfg_float(config, "tree_support_branch_angle") * M_PI / 180.0;
        let wall_count: i32 = 1.max(cfg_int(config, "tree_support_wall_count"));
        let tan_angle = angle.tan(); // when nodes are thick, they can move further. this is the max angle
        let max_move_distance: CoordF = if angle < M_PI / 2.0 { (tan_angle * layer_height) * wall_count as f64 } else { CoordF::MAX };
        let max_move_distance2: f64 = max_move_distance * max_move_distance;
        let _tip_layers: usize = (self.base_radius / layer_height) as usize; //The number of layers to be shrinking the circle to create a tip. This produces a 45 degree angle.
        let radius_sample_resolution: CoordF = self.m_ts_data.m_radius_sample_resolution;
        let support_on_buildplate_only = cfg_bool(config, "support_on_build_plate_only");
        let _top_interface_layers: usize = cfg_int(config, "support_interface_top_layers") as usize;
        SupportNode::set_diameter_angle_scale_factor(self.diameter_angle_scale_factor);
        // enabled only when the wall count is set to auto or dual-wall is explicitly enabled by the user.
        let tree_support_wall_count = cfg_int(config, "tree_support_wall_count");
        let bottom_expand_enabled = tree_support_wall_count > 1 || tree_support_wall_count < 0;
        let do_not_mover_under_mm: f64 = self.do_not_mover_under_mm as f64;
        let num_top_interface_layers: usize = self.m_support_params.num_top_interface_layers;
        let is_strong = self.is_strong;

        if self.m_ts_data.layer_heights.is_empty() {
            return;
        }

        // precalculate avoidance of all possible radii.
        // This will cause computing more (radius, layer_nr) pairs, but it's worth to do so since we are doning this in parallel.
        {
            // get all the possible radiis
            let n = self.contact_nodes.len();
            let mut all_layer_radius: Vec<std::collections::BTreeSet<MapKeyF64>> = vec![Default::default(); n];
            let mut all_layer_node_dist: Vec<std::collections::BTreeSet<MapKeyF64>> = vec![Default::default(); n];
            let mut layer_nr = n - 1;
            while layer_nr > 0 {
                for &p_node in &self.contact_nodes[layer_nr] {
                    all_layer_node_dist[layer_nr].insert(MapKeyF64(n!(p_node).dist_mm_to_top));
                }
                let layer_nr_next = layer_nr - 1;
                if layer_nr_next <= n - 1 && layer_nr_next > 0 {
                    let dists: Vec<f64> = all_layer_node_dist[layer_nr].iter().map(|d| d.0).collect();
                    for node_dist in dists {
                        all_layer_node_dist[layer_nr_next].insert(MapKeyF64(node_dist + self.m_ts_data.layer_heights[layer_nr].height));
                    }
                }
                let dists: Vec<f64> = all_layer_node_dist[layer_nr].iter().map(|d| d.0).collect();
                for node_dist in dists {
                    all_layer_radius[layer_nr].insert(MapKeyF64(self.calc_radius(node_dist)));
                }
                layer_nr -= 1;
            }
            // parallel pre-compute avoidance
            for layer_nr in 0..n - 1 {
                for node_radius in &all_layer_radius[layer_nr] {
                    let obj_layer_nr = self.m_ts_data.layer_heights[layer_nr].obj_layer_nr;
                    self.m_ts_data.get_avoidance(node_radius.0, obj_layer_nr, 0);
                    self.get_collision(0.0, obj_layer_nr);
                    self.get_collision(node_radius.0, obj_layer_nr);
                }
            }
        }

        self.m_spanning_trees.resize(self.contact_nodes.len(), Vec::new());

        let mut layer_nr = self.contact_nodes.len() - 1;
        // Skip layer 0, since we can't drop down the vertices there.
        while layer_nr > 0 {
            let this_layer_nr = layer_nr;
            layer_nr -= 1;
            let layer_nr = this_layer_nr;
            if (self.canceled)() {
                break;
            }
            if self.contact_nodes[layer_nr].is_empty() {
                continue;
            }

            let layer_nr_next = layer_nr - 1;
            let _print_z: CoordF = self.m_ts_data.layer_heights[layer_nr].print_z;
            let print_z_next: CoordF = self.m_ts_data.layer_heights[layer_nr_next].print_z;
            let height_next: CoordF = self.m_ts_data.layer_heights[layer_nr_next].height;
            let obj_layer_nr: usize = self.m_ts_data.layer_heights[layer_nr].obj_layer_nr;
            let obj_layer_nr_next: usize = self.m_ts_data.layer_heights[layer_nr_next].obj_layer_nr;
            // All nodes that are leaves on this layer that would result in unsupported ('mid-air') branches.
            let mut unsupported_branch_leaves: std::collections::VecDeque<(usize, NodeId)> = std::collections::VecDeque::new();

            let layer_contours: Polygons = self.m_ts_data.get_contours_with_holes(obj_layer_nr);
            let mut mst_line_x_layer_contour_cache: HashMap<(Point, Point), bool> = HashMap::new();

            //Group together all nodes for each part.
            let parts: ExPolygons = self.m_ts_data.m_layer_outlines_below[obj_layer_nr].clone();
            //All nodes that aren't inside a part get grouped together in the 0th part.
            let mut nodes_per_part: Vec<HashMap<Point, NodeId>> = vec![HashMap::new(); 1 + parts.len()];
            let layer_contact_nodes = self.contact_nodes[layer_nr].clone();
            for &p_node in &layer_contact_nodes {
                if !n!(p_node).to_buildplate {
                    // Can't rest on model and unable to reach the build plate. Then we must drop the node and leave parts unsupported.
                    if support_on_buildplate_only {
                        unsupported_branch_leaves.push_front((layer_nr, p_node));
                        continue;
                    }

                    //If permitted to fall on the model, downward growth ceases.
                    let node_radius = n!(p_node).radius;
                    let node_position = n!(p_node).position;
                    let collision0 = self.get_collision(0.0, obj_layer_nr);
                    let overlap_with_circle = shrink_ex(&collision0, scale_(node_radius) as f32);
                    if !overlap_with_circle.is_empty() && is_inside_ex(&overlap_with_circle, &node_position) {
                        continue;
                    }
                    let mut circle = crate::geometry::make_circle(scale_(node_radius), 0.00789 * scale_(node_radius));
                    circle.translate(node_position);
                    let area = avoid_object_remove_extra_small_parts(&ExPolygon::new(circle), &collision0);
                    if !area.is_empty() {
                        n!(p_node).overhang = area[0].clone();
                        let parent = n!(p_node).parent.expect("a node that cannot reach the build plate has a parent");
                        if area[0].area() < sq(scale_(1.0))
                            || (!n!(parent).to_buildplate
                                && !crate::ex_polygon::overlaps_expolys(
                                    std::slice::from_ref(&n!(p_node).overhang),
                                    std::slice::from_ref(&n!(parent).overhang),
                                ))
                        {
                            n!(p_node).valid = false;
                            n!(p_node).is_processed = true;
                            continue;
                        }
                    } else {
                        continue;
                    }
                }
                if n!(p_node).to_buildplate || parts.is_empty() {
                    //It's outside, so make it go towards the build plate.
                    if n!(p_node).type_ == TreeNodeType::ECircle && nodes_per_part[0].contains_key(&n!(p_node).position) {
                        let p = n!(p_node).position;
                        n!(p_node).position = Point::new(p.x + 1, p.y + 1);
                    }
                    nodes_per_part[0].insert(n!(p_node).position, p_node);
                    continue;
                }

                // Find which part this node is located in and group the nodes in the same part together.
                // Since nodes have a radius and the avoidance areas are offset by that radius, the set of parts
                // may be different per node. Here we consider a node to be inside the part that is closest.
                let node_position = n!(p_node).position;
                let mut closest_part_distance2: CoordF = CoordF::MAX;
                let mut closest_part: usize = usize::MAX;
                for part_index in 0..parts.len() {
                    //If it's inside, the distance is 0 and this part is considered the best.
                    if is_inside_ex_poly(&parts[part_index], &node_position) {
                        closest_part = part_index;
                        closest_part_distance2 = 0.0;
                        break;
                    }
                    let closest_point = polygon_closest_vertex(&parts[part_index].contour, &node_position).unwrap_or(Point::new(0, 0));
                    let distance2: CoordF =
                        vsize2_with_unscale(Point::new(node_position.x - closest_point.x, node_position.y - closest_point.y));
                    if distance2 < closest_part_distance2 {
                        closest_part_distance2 = distance2;
                        closest_part = part_index;
                    }
                }
                let _ = closest_part_distance2;

                //Put it in the best one. Index + 1 because the 0th index is the outside part.
                let part_slot = closest_part.wrapping_add(1);
                if n!(p_node).type_ == TreeNodeType::ECircle && nodes_per_part[part_slot].contains_key(&n!(p_node).position) {
                    let p = n!(p_node).position;
                    n!(p_node).position = Point::new(p.x + 1, p.y + 1);
                }
                nodes_per_part[part_slot].insert(n!(p_node).position, p_node);
            }

            //Create a MST for every part.
            profiler().tic();
            let mut spanning_trees: Vec<MinimumSpanningTree> = Vec::new();
            for group in &nodes_per_part {
                let mut points_to_buildplate: Vec<Point> = group.keys().copied().collect();
                // the keys are distinct, so the order is the one order by (x, y)
                points_to_buildplate.sort_by(|a, b| if a.x != b.x { a.x.cmp(&b.x) } else { a.y.cmp(&b.y) });
                spanning_trees.push(MinimumSpanningTree::from_vertices(points_to_buildplate));
            }
            profiler().stage_add(TreeSupportStage::StageMinimumSpanningTree);

            for group_index in 0..nodes_per_part.len() {
                let nodes_this_part = nodes_per_part[group_index].clone();
                let mst = &spanning_trees[group_index];
                let mut nodes_vec: Vec<(Point, NodeId)> = nodes_this_part.iter().map(|(k, v)| (*k, *v)).collect();
                nodes_vec.sort_by(|a, b| if a.0.x != b.0.x { a.0.x.cmp(&b.0.x) } else { a.0.y.cmp(&b.0.y) });
                //In the first pass, merge all nodes that are close together.
                let mut lower_avoid_p1: Arc<ExPolygons> = Arc::new(Vec::new());
                if group_index == 0 {
                    let lookahead_layers: i32 = 1.max((8.0 / height_next.max(0.1)).round() as i32);
                    let lower_obj_layer: usize =
                        if obj_layer_nr_next > lookahead_layers as usize { obj_layer_nr_next - lookahead_layers as usize } else { 0 };
                    lower_avoid_p1 = self.get_avoidance(TS_MIN_BRANCH_RADIUS, lower_obj_layer);
                }
                for entry in &nodes_vec {
                    let p_node = entry.1;
                    if !n!(p_node).valid {
                        continue;
                    }
                    if n!(p_node).fading {
                        continue;
                    }
                    let node_position = n!(p_node).position;
                    let neighbours: Vec<Point> = mst.adjacent_nodes(node_position);
                    'first_pass_next: {
                        if n!(p_node).type_ == TreeNodeType::EPolygon {
                            // Remove all circle neighbours that are completely inside the polygon and merge them into this node.
                            for neighbour in &neighbours {
                                let neighbour_node = nodes_this_part[neighbour];
                                let mut can_merge = false;
                                if !n!(neighbour_node).valid {
                                    continue;
                                }
                                if n!(neighbour_node).fading {
                                    continue;
                                }
                                if n!(neighbour_node).type_ == TreeNodeType::EPolygon {
                                    let node_dtt = n!(p_node).distance_to_top;
                                    let nb_dtt = n!(neighbour_node).distance_to_top;
                                    // `int > size_t`: compared as unsigned
                                    if (node_dtt < 0 && nb_dtt < 0)
                                        || ((node_dtt as i64 as usize) > num_top_interface_layers + 1
                                            && (nb_dtt as i64 as usize) > num_top_interface_layers + 1)
                                    {
                                        let node_overhang = n!(p_node).overhang.clone();
                                        let nb_overhang = n!(neighbour_node).overhang.clone();
                                        let overhang_shrinked =
                                            shrink_ex(std::slice::from_ref(&node_overhang), scale_(support_extrusion_width) as f32);
                                        if !overhang_shrinked.is_empty()
                                            && crate::ex_polygon::overlaps_expolys(&overhang_shrinked, std::slice::from_ref(&nb_overhang))
                                        {
                                            let mut tmp = union_ex(&[node_overhang, nb_overhang]);
                                            if !tmp.is_empty() && tmp.len() == 1 {
                                                let next_pt = polygon_centroid(&tmp[0].contour);
                                                let roof = n!(p_node).support_roof_layers_below.max(n!(neighbour_node).support_roof_layers_below) - 1;
                                                let next_node = self.m_ts_data.create_node(
                                                    next_pt,
                                                    node_dtt.max(nb_dtt) + 1,
                                                    obj_layer_nr_next as i32,
                                                    roof,
                                                    true,
                                                    Some(p_node),
                                                    print_z_next,
                                                    height_next,
                                                    0.0,
                                                    0.0,
                                                );
                                                n!(next_node).max_move_dist = 0.0;
                                                n!(next_node).overhang = tmp.remove(0);
                                                n!(next_node).target_radius = n!(p_node).target_radius.max(n!(neighbour_node).target_radius);
                                                n!(next_node).origin_area = n!(next_node).overhang.area();
                                                let nb_mass = n!(neighbour_node).total_carried_mass(&self.m_ts_data.contact_nodes);
                                                n!(next_node).carried_mass += nb_mass;
                                                self.contact_nodes[layer_nr_next].push(next_node);
                                                n!(p_node).valid = false;
                                                n!(neighbour_node).valid = false;
                                                break 'first_pass_next;
                                            }
                                        }
                                    }
                                } else {
                                    let neighbour_radius: Coord = scale_(n!(neighbour_node).radius) as Coord;
                                    let pt_north = Point::new(neighbour.x, neighbour.y + neighbour_radius);
                                    let pt_south = Point::new(neighbour.x, neighbour.y - neighbour_radius);
                                    let pt_west = Point::new(neighbour.x - neighbour_radius, neighbour.y);
                                    let pt_east = Point::new(neighbour.x + neighbour_radius, neighbour.y);
                                    let node_overhang = n!(p_node).overhang.clone();
                                    can_merge = is_inside_ex_poly(&node_overhang, neighbour)
                                        && is_inside_ex_poly(&node_overhang, &pt_north)
                                        && is_inside_ex_poly(&node_overhang, &pt_south)
                                        && is_inside_ex_poly(&node_overhang, &pt_west)
                                        && is_inside_ex_poly(&node_overhang, &pt_east);
                                    if !can_merge && is_inside_ex_poly(&node_overhang, neighbour) {
                                        n!(neighbour_node).fading = true;
                                        if bbox_radius(&node_overhang.contour.bounding_box()) < neighbour_radius as f64 {
                                            n!(p_node).target_radius = n!(neighbour_node).radius;
                                        }
                                    }
                                }
                                if n!(p_node).valid && can_merge {
                                    let nb_merged = n!(neighbour_node).merged_neighbours.clone();
                                    n!(p_node).merged_neighbours.insert(0, neighbour_node);
                                    n!(p_node).merged_neighbours.extend(nb_merged);
                                    n!(neighbour_node).valid = false;
                                }
                            }
                        } else if neighbours.len() == 1
                            && vsize2_with_unscale(Point::new(neighbours[0].x - node_position.x, neighbours[0].y - node_position.y))
                                < self.get_max_move_dist(p_node, tan_angle, 2)
                            && mst.adjacent_nodes(neighbours[0]).len() == 1
                            && n!(nodes_this_part[&neighbours[0]]).type_ != TreeNodeType::EPolygon
                        {
                            // We have just two nodes left, and they're very close, and the only neighbor is not ePolygon
                            //Insert a completely new node and let both original nodes fade.
                            let neighbour = nodes_this_part[&neighbours[0]];
                            //Average position of the two nodes.
                            let mid_position = point_div(Point::new(node_position.x + neighbours[0].x, node_position.y + neighbours[0].y), 2);
                            let mut next_position = mid_position;
                            let next_radius: CoordF = self.calc_radius(n!(p_node).dist_mm_to_top + height_next);
                            let avoid_layer = self.get_avoidance(next_radius, obj_layer_nr_next);
                            {
                                let m_self: f64 = (n!(p_node).total_carried_mass(&self.m_ts_data.contact_nodes) as f64).max(EPSILON);
                                let m_nb: f64 = (n!(neighbour).total_carried_mass(&self.m_ts_data.contact_nodes) as f64).max(EPSILON);
                                let mut nb_weight = m_nb / (m_self + m_nb);
                                if group_index == 0 && !lower_avoid_p1.is_empty() {
                                    let me_in_low = is_inside_ex(&lower_avoid_p1, &node_position);
                                    let nb_in_low = is_inside_ex(&lower_avoid_p1, &neighbours[0]);
                                    if !me_in_low && nb_in_low {
                                        nb_weight = nb_weight.min(0.2);
                                    } else if me_in_low && !nb_in_low {
                                        nb_weight = nb_weight.max(0.8);
                                    }
                                }
                                let weighted_pos = Point::new(
                                    node_position.x + (nb_weight * (neighbours[0].x - node_position.x) as f64).round() as Coord,
                                    node_position.y + (nb_weight * (neighbours[0].y - node_position.y) as f64).round() as Coord,
                                );
                                if !is_inside_ex(&self.get_collision(0.0, obj_layer_nr_next), &weighted_pos)
                                    && (group_index > 0 || !is_inside_ex(&avoid_layer, &weighted_pos))
                                {
                                    next_position = weighted_pos;
                                }
                            }
                            if group_index == 0 {
                                //Avoid collisions.
                                //100 micron extra for rounding errors.
                                let max_move_between_samples: CoordF = max_move_distance + radius_sample_resolution + EPSILON;
                                move_out_expolys(&avoid_layer, &mut next_position, radius_sample_resolution + EPSILON, max_move_between_samples);
                            }

                            let node_parent: NodeId = if n!(p_node).parent.is_some() && n!(neighbour).parent.is_some() {
                                if n!(p_node).radius >= n!(neighbour).radius {
                                    p_node
                                } else {
                                    neighbour
                                }
                            } else if n!(p_node).parent.is_some() {
                                p_node
                            } else {
                                neighbour
                            };
                            // Make sure the next pass doesn't drop down either of these (since that already happened).
                            let other = if node_parent == p_node { neighbour } else { p_node };
                            n!(node_parent).merged_neighbours.insert(0, other);
                            let to_buildplate = !is_inside_ex(&self.get_collision(0.0, obj_layer_nr_next), &next_position);
                            let (parent_dtt, parent_roof) = (n!(node_parent).distance_to_top, n!(node_parent).support_roof_layers_below);
                            let next_node = self.m_ts_data.create_node(
                                next_position,
                                parent_dtt + 1,
                                obj_layer_nr_next as i32,
                                parent_roof - 1,
                                to_buildplate,
                                Some(node_parent),
                                print_z_next,
                                height_next,
                                0.0,
                                0.0,
                            );
                            let dist_xy_to_node = point_dist(next_position, n!(p_node).position);
                            let dist_xy_to_neighbor = point_dist(next_position, n!(neighbour).position);
                            n!(next_node).orig_pos = if dist_xy_to_neighbor < dist_xy_to_node { n!(p_node).orig_pos } else { n!(neighbour).orig_pos };
                            self.get_max_move_dist(next_node, tan_angle, 1);
                            self.contact_nodes[layer_nr_next].push(next_node);
                            n!(neighbour).valid = false;
                            n!(p_node).valid = false;
                        } else if neighbours.len() > 1 {
                            //Don't merge leaf nodes because we would then incur movement greater than the maximum move distance.
                            //Remove all neighbours that are too close and merge them into this node.
                            for neighbour in &neighbours {
                                if vsize2_with_unscale(Point::new(neighbour.x - node_position.x, neighbour.y - node_position.y))
                                    < self.get_max_move_dist(p_node, tan_angle, 2)
                                {
                                    let neighbour_node = nodes_this_part[neighbour];
                                    if !n!(neighbour_node).valid {
                                        continue;
                                    }
                                    if n!(neighbour_node).type_ == TreeNodeType::EPolygon {
                                        continue;
                                    }
                                    // bug fix: STUDIO-6326, do not merge if current node has smaller radius than neighbor.
                                    if n!(p_node).radius < n!(neighbour_node).radius {
                                        continue;
                                    }
                                    if n!(p_node).valid {
                                        let nb_merged = n!(neighbour_node).merged_neighbours.clone();
                                        n!(p_node).merged_neighbours.insert(0, neighbour_node);
                                        n!(p_node).merged_neighbours.extend(nb_merged);
                                        n!(neighbour_node).valid = false;
                                    }
                                }
                            }
                        }
                    }
                }

                //In the second pass, move all middle nodes.
                for entry in &nodes_vec {
                    let p_node = entry.1;
                    if !n!(p_node).valid {
                        continue;
                    }
                    if n!(p_node).fading {
                        let next_radius: CoordF = n!(p_node).radius - max_move_distance;
                        if next_radius < EPSILON {
                            continue;
                        }
                        let (pos, dtt, roof, tb) =
                            (n!(p_node).position, n!(p_node).distance_to_top, n!(p_node).support_roof_layers_below, n!(p_node).to_buildplate);
                        let next_node = self.m_ts_data.create_node(
                            pos,
                            dtt + 1,
                            obj_layer_nr_next as i32,
                            roof - 1,
                            tb,
                            Some(p_node),
                            print_z_next,
                            height_next,
                            0.0,
                            0.0,
                        );
                        n!(next_node).max_move_dist = 0.0;
                        n!(next_node).radius = next_radius;
                        n!(next_node).fading = true;
                        self.contact_nodes[layer_nr_next].push(next_node);
                        continue;
                    }
                    if n!(p_node).type_ == TreeNodeType::EPolygon {
                        // polygon node do not merge or move
                        if n!(p_node).overhang.contour.points.is_empty() {
                            n!(p_node).valid = false;
                            continue;
                        }
                        let to_buildplate = true;
                        let node_overhang = n!(p_node).overhang.clone();
                        let mut overhangs_next: ExPolygons = vec![node_overhang.clone()];
                        let poly_radius = bbox_radius(&node_overhang.contour.bounding_box());
                        let mut offseted = false;
                        let node_target_radius = n!(p_node).target_radius;
                        // polygon support expansion cases:
                        // 1. bottom expansion is enabled
                        // 2. large tree supports merge into the region
                        if (bottom_expand_enabled && n!(p_node).print_z < do_not_mover_under_mm)
                            || (poly_radius < scale_(node_target_radius) || node_overhang.area() < 3.0 * sq(scale_(node_target_radius)))
                        {
                            // `offset_ex({node.overhang}, ...)`: the overload of one ExPolygon
                            overhangs_next = offset_ex_expolygon(&node_overhang, scale_(max_move_distance / 2.0) as f32);
                            offseted = true;
                        }
                        // keep only the part that won't be removed by the next layer
                        let collision_next = self.get_collision(0.0, obj_layer_nr_next);
                        overhangs_next = diff_clipped(&overhangs_next, &collision_next);
                        let node_dtt = n!(p_node).distance_to_top;
                        if node_dtt == 0 {
                            overhangs_next = offset2_ex(&overhangs_next, scale_(max_move_distance) as f32, -scale_(max_move_distance) as f32);
                            n!(p_node).origin_area = node_overhang.area();
                            densify_polygon(&mut n!(p_node).overhang.contour, 2.0);
                        }
                        let node_roof = n!(p_node).support_roof_layers_below;
                        if num_top_interface_layers > 0
                            && obj_layer_nr_next > 0
                            && node_roof == 1
                            && (node_dtt as i64 as usize) >= num_top_interface_layers
                        {
                            overhangs_next = safe_offset_inc(
                                &overhangs_next,
                                scale_(max_move_distance) as Coord,
                                &collision_next,
                                scale_(TS_MIN_BRANCH_RADIUS * 1.75) as Coord,
                                0,
                                1,
                            );
                        }
                        let node_origin_area = n!(p_node).origin_area;
                        for mut overhang in overhangs_next {
                            if overhang.contour.points.is_empty() {
                                continue;
                            }
                            if overhang.area() > node_origin_area / 2.0 && overhang.area() > sq(scale_(10.0)) {
                                let contour = overhang.contour.clone();
                                let mut movements: HashMap<Point, Point> = HashMap::new();
                                smooth_filter(&overhang.contour, 2, &mut movements, max_move_distance / 2.0);
                                let contour_expolys = vec![ExPolygon::new(contour)];
                                for pt in overhang.contour.points.iter_mut() {
                                    let mv = movements[pt];
                                    let tmp = Point::new(pt.x + mv.x, pt.y + mv.y);
                                    if !is_inside_ex(&contour_expolys, &tmp) && !is_inside_ex(&self.m_ts_data.m_layer_outlines_below[obj_layer_nr], &tmp) {
                                        *pt = tmp;
                                    }
                                }
                            }
                            // if the part would fall straight to th buildplate, shrink it a little
                            if !offseted && node_roof < 0 && overhang.area() > node_origin_area / 2.0 && overhang.area() > sq(scale_(10.0)) {
                                let shrink_overhangs = union_ex(&shrink_ex(
                                    &safe_union(std::slice::from_ref(&overhang), &[]),
                                    scale_(max_move_distance / 2.0) as f32,
                                ));
                                if shrink_overhangs.len() == 1
                                    && shrink_overhangs[0].area() > sq(scale_(10.0))
                                    && !crate::ex_polygon::overlaps_expolys(
                                        std::slice::from_ref(&overhang),
                                        &self.m_ts_data.m_layer_outlines_below[obj_layer_nr_next],
                                    )
                                {
                                    if diff_ex(
                                        std::slice::from_ref(&overhang),
                                        &offset_ex_expolygon(&shrink_overhangs[0], scale_(max_move_distance) as f32),
                                    )
                                    .is_empty()
                                    {
                                        overhang = shrink_overhangs[0].clone();
                                    }
                                }
                            }
                            // the two arms of the C++ differ in the order of the assignments only
                            let next_pt = polygon_centroid(&overhang.contour);
                            let next_node = self.m_ts_data.create_node(
                                next_pt,
                                node_dtt + 1,
                                obj_layer_nr_next as i32,
                                node_roof - 1,
                                to_buildplate,
                                Some(p_node),
                                print_z_next,
                                height_next,
                                0.0,
                                0.0,
                            );
                            n!(next_node).max_move_dist = 0.0;
                            n!(next_node).overhang = overhang;
                            n!(next_node).target_radius = node_target_radius;
                            n!(next_node).origin_area = node_origin_area;
                            self.contact_nodes[layer_nr_next].push(next_node);
                        }
                        continue;
                    }

                    let node_position = n!(p_node).position;
                    //If the branch falls completely inside a collision area (the entire branch would be removed by the X/Y offset), delete it.
                    if group_index > 0 && is_inside_ex(&self.get_collision(0.0, obj_layer_nr), &node_position) {
                        let branch_radius_node: CoordF = self.get_radius(p_node);
                        let to_outside = projection_onto(&self.get_collision(0.0, obj_layer_nr), &node_position);
                        let dist2_to_outside = vsize2_with_unscale(Point::new(node_position.x - to_outside.x, node_position.y - to_outside.y));
                        if dist2_to_outside >= branch_radius_node * branch_radius_node {
                            //Too far inside.
                            if support_on_buildplate_only {
                                unsupported_branch_leaves.push_front((layer_nr, p_node));
                            } else {
                                n!(p_node).valid = false;
                            }
                            continue;
                        }
                        // if the link between parent and current is cut by contours, mark current as bottom contact node
                        if let Some(parent) = n!(p_node).parent {
                            if !intersection_ln(&Line::new(n!(p_node).position, n!(parent).position), &layer_contours).is_empty() {
                                n!(p_node).valid = false;
                                continue;
                            }
                        }
                    }
                    let mut next_layer_vertex = node_position;
                    let mut move_to_neighbor_center = Point::new(0, 0);
                    let mut move_to_neighbor_center_base = Point::new(0, 0);

                    let neighbours: Vec<Point> = mst.adjacent_nodes(node_position);
                    // 1. do not merge neighbors under 5mm
                    // 2. Only merge node with single neighbor in distance between [max_move_distance, 10mm/layer_height]
                    let dist2_to_first_neighbor: f32 = if neighbours.is_empty() {
                        0.0
                    } else {
                        vsize2_with_unscale(Point::new(neighbours[0].x - node_position.x, neighbours[0].y - node_position.y)) as f32
                    };
                    let node_print_z = n!(p_node).print_z;
                    let node_dist_mm_to_top = n!(p_node).dist_mm_to_top;
                    // Only nodes that aren't about to collapse.
                    if node_print_z > do_not_mover_under_mm
                        && (neighbours.len() > 1
                            || (neighbours.len() == 1 && dist2_to_first_neighbor as f64 >= self.get_max_move_dist(p_node, tan_angle, 2)))
                    {
                        // Move towards the average position of all neighbours, weighted by carried_mass
                        // to encourage light branches to move towards heavy trunks and suppress heavy trunks
                        // from deflecting towards light branches.
                        let self_mass: f64 = (n!(p_node).total_carried_mass(&self.m_ts_data.contact_nodes) as f64).max(EPSILON);
                        let mut max_nb_mass: f64 = 0.0;
                        let prev_move = match n!(p_node).parent {
                            Some(parent) => Point::new(node_position.x - n!(parent).position.x, node_position.y - n!(parent).position.y),
                            None => Point::new(0, 0),
                        };
                        let prev_move_d = (prev_move.x as f64, prev_move.y as f64);
                        let has_prev_move = prev_move_d.0 * prev_move_d.0 + prev_move_d.1 * prev_move_d.1 > scale_(0.01) * scale_(0.01);
                        // Direction towards the nearest edge of the model outlines below (for group_index == 0).
                        // Pulling a branch deeper over a lower model part forces it to either reverse outward later
                        // (elbow bend) or land on the model top.
                        let mut outlines_below_out_d = (0.0_f64, 0.0_f64);
                        let mut in_outlines_below = false;
                        if group_index == 0 && !parts.is_empty() && !lower_avoid_p1.is_empty() && is_inside_ex(&parts, &node_position) {
                            let proj_out = projection_onto(&parts, &node_position);
                            outlines_below_out_d = ((proj_out.x - node_position.x) as f64, (proj_out.y - node_position.y) as f64);
                            in_outlines_below = outlines_below_out_d.0 * outlines_below_out_d.0 + outlines_below_out_d.1 * outlines_below_out_d.1
                                > scale_(0.2) * scale_(0.2);
                        }
                        let mut base_dir_d = (0.0_f64, 0.0_f64);
                        let mut sum_dir_d = (0.0_f64, 0.0_f64);
                        for neighbour in &neighbours {
                            // do not move to the neighbor to be deleted
                            let neighbour_node = nodes_this_part[neighbour];
                            if !n!(neighbour_node).valid {
                                continue;
                            }
                            let direction: Point;
                            if n!(neighbour_node).type_ == TreeNodeType::EPolygon && n!(neighbour_node).overhang.is_valid() {
                                let contact_point = projection_onto(std::slice::from_ref(&n!(neighbour_node).overhang), &node_position);
                                direction = Point::new(contact_point.x - node_position.x, contact_point.y - node_position.y);
                            } else {
                                direction = Point::new(neighbour.x - node_position.x, neighbour.y - node_position.y);
                            }
                            // do not move to neighbor that's too far away
                            let dist2_to_neighbor: f32 = vsize2_with_unscale(direction) as f32;

                            let branch_bottom_radius: CoordF = self.calc_radius(node_dist_mm_to_top + node_print_z);
                            let neighbour_bottom_radius: CoordF = self.calc_radius(n!(neighbour_node).dist_mm_to_top + n!(neighbour_node).print_z);
                            let max_converge_distance: f64 =
                                tan_angle.mul_add(node_print_z - do_not_mover_under_mm, branch_bottom_radius.max(neighbour_bottom_radius));
                            if dist2_to_neighbor as f64 > max_converge_distance * max_converge_distance {
                                continue;
                            }

                            if is_line_cut_by_contour(&mut mst_line_x_layer_contour_cache, &layer_contours, node_position, *neighbour) {
                                continue;
                            }

                            let mut mass_weight: f64;
                            let nb_mass: f64 = (n!(neighbour_node).total_carried_mass(&self.m_ts_data.contact_nodes) as f64).max(EPSILON);
                            max_nb_mass = max_nb_mass.max(nb_mass);
                            {
                                let dir_d = (direction.x as f64, direction.y as f64);
                                let reverses_prev = has_prev_move && dir_d.0 * prev_move_d.0 + dir_d.1 * prev_move_d.1 < 0.0;
                                if nb_mass >= self_mass {
                                    mass_weight = (2.0 * nb_mass / (self_mass + nb_mass)).sqrt();
                                } else if reverses_prev {
                                    mass_weight = (2.0 * nb_mass / (self_mass + nb_mass)).sqrt();
                                } else {
                                    mass_weight = 1.0;
                                }
                                if group_index == 0 && !lower_avoid_p1.is_empty() {
                                    let me_in_low = is_inside_ex(&lower_avoid_p1, &node_position);
                                    let nb_in_low = is_inside_ex(&lower_avoid_p1, neighbour);
                                    if !me_in_low && nb_in_low {
                                        mass_weight *= 0.25;
                                    } else if me_in_low && !nb_in_low {
                                        mass_weight = mass_weight.max(1.4);
                                    } else if me_in_low && nb_in_low && in_outlines_below {
                                        if dir_d.0 * outlines_below_out_d.0 + dir_d.1 * outlines_below_out_d.1 > 0.0 {
                                            mass_weight = mass_weight.max(1.3);
                                        } else {
                                            mass_weight *= 0.4;
                                        }
                                    }
                                }
                            }

                            if !is_strong {
                                let w = mass_weight / dist2_to_neighbor as f64;
                                sum_dir_d = (sum_dir_d.0 + direction.x as f64 * w, sum_dir_d.1 + direction.y as f64 * w);
                                let wb = 1.0 / dist2_to_neighbor as f64;
                                base_dir_d = (base_dir_d.0 + direction.x as f64 * wb, base_dir_d.1 + direction.y as f64 * wb);
                            } else {
                                sum_dir_d = (sum_dir_d.0 + direction.x as f64 * mass_weight, sum_dir_d.1 + direction.y as f64 * mass_weight);
                                base_dir_d = (base_dir_d.0 + direction.x as f64, base_dir_d.1 + direction.y as f64);
                            }
                        }

                        let sum_direction = Point::new(sum_dir_d.0.round() as Coord, sum_dir_d.1.round() as Coord);
                        let base_direction = Point::new(base_dir_d.0.round() as Coord, base_dir_d.1.round() as Coord);
                        if !is_strong {
                            move_to_neighbor_center = sum_direction;
                            move_to_neighbor_center_base = base_direction;
                        } else {
                            if vsize2_with_unscale(sum_direction) <= self.get_max_move_dist(p_node, tan_angle, 2) {
                                move_to_neighbor_center = sum_direction;
                            } else {
                                move_to_neighbor_center = normal(sum_direction, scale_(self.get_max_move_dist(p_node, tan_angle, 1)));
                            }
                            if vsize2_with_unscale(base_direction) <= self.get_max_move_dist(p_node, tan_angle, 2) {
                                move_to_neighbor_center_base = base_direction;
                            } else {
                                move_to_neighbor_center_base = normal(base_direction, scale_(self.get_max_move_dist(p_node, tan_angle, 1)));
                            }
                        }
                        // When all reachable neighbours carry less mass than this node and pull it opposite to its
                        // trajectory (or deeper over lower model outlines), cap the attraction step by mass ratio.
                        if max_nb_mass > 0.0 && max_nb_mass < self_mass {
                            let move_d = (move_to_neighbor_center.x as f64, move_to_neighbor_center.y as f64);
                            let rev_move = (has_prev_move && move_d.0 * prev_move_d.0 + move_d.1 * prev_move_d.1 < 0.0)
                                || (in_outlines_below && move_d.0 * outlines_below_out_d.0 + move_d.1 * outlines_below_out_d.1 < 0.0);
                            if rev_move {
                                let attract_scale: f64 = if support_on_buildplate_only || is_strong { 1.0 } else { 2.0 };
                                let mass_factor: f64 = std_clamp(2.0 * max_nb_mass / (self_mass + max_nb_mass), 0.15, 1.0);
                                let max_attract: f64 = (self.get_max_move_dist(p_node, tan_angle, 1) * mass_factor) / attract_scale;
                                if vsize2_with_unscale(move_to_neighbor_center) > sq(max_attract) {
                                    move_to_neighbor_center = normal(move_to_neighbor_center, scale_(max_attract));
                                }
                            }
                        }
                    }

                    let next_radius: CoordF = self.calc_radius(node_dist_mm_to_top + height_next);
                    let avoidance_next = self.get_avoidance(next_radius, obj_layer_nr_next);

                    let mut to_outside = projection_onto(&avoidance_next, &node_position);
                    let mut direction_to_outer = Point::new(to_outside.x - node_position.x, to_outside.y - node_position.y);
                    let skin_direction = n!(p_node).skin_direction;
                    if skin_direction != Point::new(0, 0) && node_dist_mm_to_top < 3.0 {
                        move_to_neighbor_center = normal(skin_direction, scale_(max_move_distance));
                        direction_to_outer = move_to_neighbor_center;
                    }
                    let mut dist2_to_outer: f64 = vsize2_with_unscale(direction_to_outer);
                    // don't move if
                    // 1) line of node and to_outside is cut by contour (means supports may intersect with object)
                    // 2) it's impossible to move to build plate
                    if is_line_cut_by_contour(&mut mst_line_x_layer_contour_cache, &layer_contours, node_position, to_outside)
                        || dist2_to_outer > max_move_distance2 * (obj_layer_nr * obj_layer_nr) as f64
                        || !is_inside_ex(&avoidance_next, &node_position)
                    {
                        // try move to outside of lower layer instead
                        let mut candidate_vertex = node_position;
                        // 100 micron extra for rounding errors.
                        let max_move_between_samples: CoordF = max_move_distance + radius_sample_resolution + EPSILON;
                        // use get_collision instead of get_avoidance here (See STUDIO-4252)
                        let is_outside = move_out_expolys(
                            &self.get_collision(next_radius, obj_layer_nr_next),
                            &mut candidate_vertex,
                            max_move_between_samples,
                            max_move_between_samples,
                        );
                        if is_outside {
                            direction_to_outer = Point::new(candidate_vertex.x - node_position.x, candidate_vertex.y - node_position.y);
                            dist2_to_outer = vsize2_with_unscale(direction_to_outer);
                        } else {
                            direction_to_outer = Point::new(0, 0);
                            dist2_to_outer = 0.0;
                        }
                    }
                    // move to the averaged direction of neighbor center and contour edge if they are roughly same direction
                    if dist2_to_outer > EPSILON && move_to_neighbor_center_base != Point::new(0, 0) {
                        // When escaping an obstacle or avoidance zone, keep the unweighted attraction magnitude
                        // so carried_mass scaling never alters the escape-vs-attraction vector balance.
                        move_to_neighbor_center = move_to_neighbor_center_base;
                    }
                    let mut movement: Point;
                    if support_on_buildplate_only {
                        // `Point * 2`: Eigen's integer product
                        movement = Point::new(move_to_neighbor_center.x + direction_to_outer.x * 2, move_to_neighbor_center.y + direction_to_outer.y * 2);
                    } else if !is_strong {
                        // `direction_to_outer * (1 / dist2_to_outer)`: the member operator* of a non-const Point (lrint)
                        let outer = if dist2_to_outer > EPSILON { point_mul(direction_to_outer, 1.0 / dist2_to_outer) } else { Point::new(0, 0) };
                        movement = Point::new(move_to_neighbor_center.x * 2 + outer.x, move_to_neighbor_center.y * 2 + outer.y);
                    } else {
                        // `movement` is a default constructed Point here, (0, 0): its dot product is 0
                        movement = Point::new(0, 0);
                        let dot = movement.x * move_to_neighbor_center.x + movement.y * move_to_neighbor_center.y;
                        if dot as f64 >= 0.2 || move_to_neighbor_center == Point::new(0, 0) {
                            movement = Point::new(direction_to_outer.x + move_to_neighbor_center.x, direction_to_outer.y + move_to_neighbor_center.y);
                        } else {
                            movement = move_to_neighbor_center; // otherwise move to neighbor center first
                        }
                    }

                    if vsize2_with_unscale(movement) > self.get_max_move_dist(p_node, tan_angle, 2) {
                        movement = normal(movement, scale_(self.get_max_move_dist(p_node, tan_angle, 1)));
                    }

                    next_layer_vertex = Point::new(next_layer_vertex.x + movement.x, next_layer_vertex.y + movement.y);

                    // `if (group_index == 0 && 0)`: never taken

                    let next_collision = self.get_collision(0.0, obj_layer_nr_next);
                    let to_buildplate = !is_inside_ex(&self.m_ts_data.m_layer_outlines[obj_layer_nr_next], &next_layer_vertex);
                    let (node_dtt, node_roof) = (n!(p_node).distance_to_top, n!(p_node).support_roof_layers_below);
                    let next_node = self.m_ts_data.create_node(
                        next_layer_vertex,
                        node_dtt + 1,
                        obj_layer_nr_next as i32,
                        node_roof - 1,
                        to_buildplate,
                        Some(p_node),
                        print_z_next,
                        height_next,
                        0.0,
                        0.0,
                    );
                    // don't increase radius if next node will collide partially with the object (STUDIO-7883)
                    to_outside = projection_onto(&next_collision, &n!(next_node).position);
                    direction_to_outer = Point::new(to_outside.x - node_position.x, to_outside.y - node_position.y);
                    let dist_to_outer: f64 = point_norm(direction_to_outer) * SCALING_FACTOR;
                    let node_radius = n!(p_node).radius;
                    let (next_print_z, next_height, next_radius_now) = (n!(next_node).print_z, n!(next_node).height, n!(next_node).radius);
                    n!(next_node).radius =
                        if bottom_expand_enabled && next_print_z < do_not_mover_under_mm && node_dist_mm_to_top > do_not_mover_under_mm {
                            node_radius + self.support_flare_step_width(next_print_z, next_height) / 2.0
                        } else {
                            node_radius.max(next_radius_now.min(dist_to_outer))
                        };
                    self.get_max_move_dist(next_node, tan_angle, 1);
                    self.contact_nodes[layer_nr_next].push(next_node);
                }

                // Sort contact_nodes for deterministic processing in the next layer
                {
                    let nodes = &self.m_ts_data.contact_nodes;
                    crate::clipper::libcxx_sort(&mut self.contact_nodes[layer_nr_next], |a: &NodeId, b: &NodeId| {
                        let (pa, pb) = (nodes[*a].position, nodes[*b].position);
                        if pa.x != pb.x {
                            return pa.x < pb.x;
                        }
                        pa.y < pb.y
                    });
                }
            }

            // BBS multi-diameter: a slab is one extrusion printed at the cross-section of its top face, so a
            // branch that drifts while crossing the slab ends up somewhere the slab below does not carry it.
            // Cap the drift a chain may accumulate between two slab tops by what the slab's own height allows.
            if self.m_support_merge_plan.enabled && layer_nr_next < self.m_ts_data.layer_heights.len() && !self.contact_nodes[layer_nr_next].is_empty() {
                let obj_layer_next = self.m_ts_data.layer_heights[layer_nr_next].obj_layer_nr;
                let slab = self.m_support_merge_plan.slab_for_layer(obj_layer_next).map(|s| (s.top_idx, s.height));
                // Nothing to cap on the slab's own top layer: that face is where the budget is handed out.
                if let Some((slab_top_idx, slab_height)) = slab {
                    if obj_layer_next != slab_top_idx && slab_height > EPSILON {
                        let budget_scaled: f64 = scale_(tan_angle * slab_height);
                        for &node in &self.contact_nodes[layer_nr_next] {
                            if n!(node).type_ == TreeNodeType::EPolygon || n!(node).parent.is_none() {
                                continue;
                            }
                            let mut anchor = node;
                            while n!(anchor).parent.is_some() && (n!(anchor).obj_layer_nr as usize) < slab_top_idx {
                                anchor = n!(anchor).parent.unwrap();
                            }
                            if anchor == node {
                                continue;
                            }
                            let delta = ((n!(node).position.x - n!(anchor).position.x) as f64, (n!(node).position.y - n!(anchor).position.y) as f64);
                            let len = (delta.0 * delta.0 + delta.1 * delta.1).sqrt();
                            if len > budget_scaled && len > EPSILON {
                                let capped = (delta.0 * (budget_scaled / len), delta.1 * (budget_scaled / len));
                                let ap = n!(anchor).position;
                                n!(node).position = Point::new(ap.x + capped.0 as Coord, ap.y + capped.1 as Coord);
                            }
                        }
                    }
                }
            }

            if layer_nr_next == 0 && support_on_buildplate_only && !self.contact_nodes[layer_nr_next].is_empty() {
                for &node in &self.contact_nodes[layer_nr_next] {
                    if !n!(node).to_buildplate {
                        unsupported_branch_leaves.push_front((layer_nr_next, node));
                    }
                }
            }

            // Prune all branches that couldn't find support on either the model or the buildplate (resulting in 'mid-air' branches).
            while !unsupported_branch_leaves.is_empty() {
                let entry = *unsupported_branch_leaves.back().unwrap();
                let mut i_node: Option<NodeId> = Some(entry.1);
                while let Some(i) = i_node {
                    let i_layer: usize = n!(i).obj_layer_nr as usize;
                    // update the parent-child chain
                    let i_child = n!(i).child;
                    let i_parent = n!(i).parent;
                    if let Some(parent) = i_parent {
                        n!(parent).child = i_child;
                        let parents = n!(i).parents.clone();
                        for parent in parents {
                            if n!(parent).child == Some(i) {
                                n!(parent).child = i_child;
                            }
                        }
                    }
                    if let Some(child) = i_child {
                        n!(child).parent = i_parent;
                        if let Some(pos) = n!(child).parents.iter().position(|&p| p == i) {
                            n!(child).parents.remove(pos);
                        }
                        if !n!(i).parents.is_empty() {
                            let parents = n!(i).parents.clone();
                            n!(child).parents.extend(parents);
                        }
                    }
                    n!(i).is_processed = true; // mark to be deleted later
                    let merged = n!(i).merged_neighbours.clone();
                    for neighbour in merged {
                        if !n!(neighbour).is_processed {
                            unsupported_branch_leaves.push_front((i_layer, neighbour));
                        }
                    }
                    i_node = n!(i).parent;
                }
                unsupported_branch_leaves.pop_back();
            }

            {
                let nodes = &self.m_ts_data.contact_nodes;
                for layer_contact_nodes in self.contact_nodes.iter_mut() {
                    if !layer_contact_nodes.is_empty() {
                        layer_contact_nodes.retain(|&node| !nodes[node].is_processed);
                    }
                }
            }

            if !self.contact_nodes.is_empty() {
                // make sure r[i - 1] - r[i] <= line_width
                let a = max_move_distance + TS_MIN_BRANCH_RADIUS;
                let radius_stable_threshold: f64 =
                    (max_move_distance + max_move_distance) / a.mul_add(a, -(TS_MIN_BRANCH_RADIUS * TS_MIN_BRANCH_RADIUS));
                let layer0 = self.contact_nodes[0].clone();
                for node0 in layer0 {
                    let mut trunk_top = node0;
                    while n!(trunk_top).parent.is_some() && n!(trunk_top).parents.len() <= 1 {
                        trunk_top = n!(trunk_top).parent.unwrap();
                    }
                    let mut node: Option<NodeId> = Some(node0);
                    while let Some(nd) = node {
                        let ref_pos = n!(trunk_top).position;
                        let stable_radius: f64 =
                            (point_dist(n!(nd).position, ref_pos).mul_add(SCALING_FACTOR, n!(nd).dist_mm_to_top) / radius_stable_threshold).sqrt();
                        if n!(nd).radius >= stable_radius {
                            break;
                        }
                        n!(nd).radius = stable_radius;
                        if n!(nd).parent.is_none() || n!(nd).parents.len() > 1 {
                            break;
                        }
                        node = n!(nd).parent;
                    }
                }
            }
        }
    }
}

// @@TAIL@@
// Keep the unused-helper warnings quiet for the not-yet-wired constants/helpers.
#[allow(dead_code)]
fn _keep_unused() {
    let _ = (sign(0.0), TAU, USE_SUPPORT_3D, NO_INDEX, sq(0.0));
    let _ = (
        MAX_BRANCH_RADIUS_FIRST_LAYER,
        MIN_BRANCH_RADIUS_FIRST_LAYER,
        M_PI,
    );
    let _ = SupportNode::NO_PARENT;
}

