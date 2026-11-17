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
use crate::ex_polygon::projection_onto;
use crate::geometry::{
    get_extents_expoly, turn90_ccw, BoundingBox, ExPolygon, ExPolygons, Line, Lines, Point, Polygon,
    Polygons,
};
use crate::minimum_spanning_tree::MinimumSpanningTree;
use crate::{Coord, CoordF};
use std::cell::RefCell;
use std::collections::{BTreeMap, HashMap, HashSet};

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
    pub m_collision_cache: RefCell<HashMap<(u64, usize), ExPolygons>>,
    pub m_avoidance_cache: RefCell<HashMap<(u64, usize), ExPolygons>>,
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
    // return polygon.contains(pt);
    polygon.contains_point(pt)
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

// ============================================================================
// BLOCKED SYMBOLS (not ported)
// ============================================================================
//
// Config-hierarchy threading is WIRED (2026-06-12): PrintObject, Print, Layer,
// LayerRegion, SlicingParameters, SupportParameters, PrintObjectConfig,
// PrintConfig, BuildVolume are all available in Rust.  The remaining blocker
// for ALL methods below is the TreeSupportData concurrent cache + TBB runtime:
//
//   TreeSupportData uses tbb::concurrent_unordered_map + tbb::spin_mutex
//   (TreeSupport.hpp:307,346-350) — no Rust equivalent ported yet.
//   Every TreeSupport class method operates on a TreeSupportData instance,
//   so they are all transitively blocked on TBB.
//
//   TreeSupport::TreeSupport(ctor)        TreeSupport.cpp:607 — blocked on
//                                          TreeSupportData/TBB node arena.
//   TreeSupport::detect_overhangs         TreeSupport.cpp:661 — blocked on
//                                          TBB parallel_for + TreeSupportData arena.
//   TreeSupport::draw_circles             TreeSupport.cpp:2284 — blocked on
//                                          TreeSupportData/TBB + SVG debug output.
//   TreeSupport::drop_nodes               TreeSupport.cpp:2853 — blocked on
//                                          TreeSupportData node arena + MST + avoidance.
//   TreeSupport::smooth_nodes (x2)        TreeSupport.cpp:3560,3647 — blocked on
//                                          TreeSupportData node arena + TreeSupport3D config.
//   TreeSupport::plan_layer_heights       TreeSupport.cpp:3766 — blocked on
//                                          TreeSupportData node arena (LayerHeightData).
//   TreeSupport::generate_contact_points  TreeSupport.cpp:3898 — blocked on
//                                          TreeSupportData node arena + detect_overhangs output.
//   TreeSupport::insert_dropped_node      TreeSupport.cpp:4156 — blocked on
//                                          TreeSupportData node arena (merge logic).
//   TreeSupport::create_node              TreeSupport.cpp:4256 — blocked on
//                                          TreeSupportData arena allocation.
//   TreeSupport::create_tree_support_layers TreeSupport.cpp:1317 — blocked on
//                                          TreeSupportData node arena + SupportLayer mutation.
//   TreeSupport::generate_toolpaths       TreeSupport.cpp:1508 — blocked on
//                                          TreeSupportData + FillLightning/SupportLayer integration.
//   TreeSupport::move_bounds_to_contact_nodes TreeSupport.cpp:1946 — blocked on
//                                          TreeSupport3D::SupportElements + TreeSupportData.
//   TreeSupport::generate                 TreeSupport.cpp:1975 — top-level driver,
//                                          blocked on all the above.
//   TreeSupport::get_trim_support_regions TreeSupport.cpp:2208 — blocked on
//                                          TreeSupportData + detect_overhangs output.
//   TreeSupport::get_avoidance/get_collision/get_collision_polys TreeSupport.cpp:2136..2187 —
//                                          blocked on TreeSupportData TBB concurrent caches.
//   TreeSupportData (ctor, get_collision, get_avoidance, calculate_*, ceil_radius, create_node, ...)
//                                          TreeSupport.hpp/TreeSupport.cpp — blocked on
//                                          tbb::concurrent_unordered_map + tbb::spin_mutex.
//   TreeSupportProfiler                   TreeSupport.cpp:91 — boost::posix_time, debug-only.
//   draw_contours_and_nodes_to_svg / draw_layer_mst  TreeSupport.cpp:187,249 — SVG debug only.
//   add_overhang                          TreeSupport.cpp:644 — blocked on TreeSupportData arena.
