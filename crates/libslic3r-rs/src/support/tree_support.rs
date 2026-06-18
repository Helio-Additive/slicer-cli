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
//! PORTING STATUS (partial): the standalone geometry helpers, math primitives,
//! data structures (SupportNode/LayerHeightData/OverhangType/...), and the branch
//! radius calculations are faithfully ported. The `TreeSupport` *class methods*
//! (detect_overhangs, draw_circles, drop_nodes, generate_contact_points,
//! plan_layer_heights, generate_toolpaths, get_trim_support_regions,
//! create_tree_support_layers, move_bounds_to_contact_nodes, generate) are blocked:
//! they require `PrintObject`/`Layer`/`Print`/`LayerRegion`, `SlicingParameters`,
//! `SupportParameters`, `FillLightning::Generator`, `TreeSupport3D`, and the
//! TBB parallel pipeline to be threaded through as usable Rust APIs, none of which
//! are available yet. See the bottom of the file for the blocked-symbol list.
//!
//! NOTE: This file is not currently wired into the module tree. The active
//! tree support implementation lives in `support/mod.rs` and `support/tree_support_3d.rs`.

use crate::ex_polygon::projection_onto;
use crate::geometry::{
    get_extents_expoly, turn90_ccw, BoundingBox, ExPolygon, ExPolygons, Line, Lines, Point, Polygon,
};
use crate::minimum_spanning_tree::MinimumSpanningTree;
use crate::{scale, unscale, Coord, CoordF};
use std::collections::HashSet;

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
    unscale(val)
}

// scale_(): mirrors libslic3r's scale_ macro.
#[inline]
fn scale_(val: f64) -> Coord {
    scale(val)
}

// Point.hpp:200,255-258: `Point operator*(const double &rhs)` returns
// `{ coord_t(x*r), coord_t(y*r) }` — `static_cast<coord_t>` TRUNCATES toward zero.
// The crate-wide `impl Mul<CoordF> for Point` uses `round_ties_even()` instead, so we
// reproduce the C++ truncation locally here for fidelity.
// FIDELITY-NOTE(F2): coord_t is int32_t; the truncation is reproduced at i64 width since
// the operand magnitudes here stay within scaled coord range.
#[inline]
fn point_mul_trunc(pt: Point, r: f64) -> Point {
    Point::new((pt.x as f64 * r) as Coord, (pt.y as f64 * r) as Coord)
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

/// TreeSupport.hpp:49-192: struct SupportNode
///
/// Represents the metadata of a node in the tree.
///
/// In C++ the tree topology is built from raw pointers (`parent`, `parents`,
/// `child`, `merged_neighbours`). Because Rust cannot faithfully represent the
/// self-referential pointer graph without an arena, the topology fields are
/// retained here as the original scalar/value metadata; the linkage fields are
/// reproduced as indices/placeholders so that callers (the not-yet-ported
/// `TreeSupportData::create_node`/`drop_nodes`) can wire them through an arena.
#[derive(Debug, Clone)]
pub struct SupportNode {
    /// TreeSupport.hpp:107-111
    /// The number of layers to go to the top of this branch.
    /// Negative value means it's a virtual node between support and overhang.
    pub distance_to_top: i32,
    // TreeSupport.hpp:112: coordf_t dist_mm_to_top = 0; // dist to bottom contact in mm
    pub dist_mm_to_top: CoordF,

    // TreeSupport.hpp:120: Point position;
    pub position: Point,
    // TreeSupport.hpp:121: Point movement;
    pub movement: Point,
    // TreeSupport.hpp:122: Point orig_pos;
    pub orig_pos: Point,
    // TreeSupport.hpp:123: mutable double radius = 0.0;
    pub radius: f64,
    // TreeSupport.hpp:124: mutable double max_move_dist = 0.0;
    pub max_move_dist: f64,
    // TreeSupport.hpp:125: TreeNodeType type = eCircle;
    pub type_: TreeNodeType,
    // TreeSupport.hpp:126: bool is_corner = false;
    pub is_corner: bool,
    // TreeSupport.hpp:127: bool is_processed = false;
    pub is_processed: bool,
    // TreeSupport.hpp:128: bool need_extra_wall = false;
    pub need_extra_wall: bool,
    // TreeSupport.hpp:129: bool is_sharp_tail = false;
    pub is_sharp_tail: bool,
    // TreeSupport.hpp:130: bool valid = true;
    pub valid: bool,
    // TreeSupport.hpp:131: bool fading = false;
    pub fading: bool,
    // TreeSupport.hpp:132: double overhang_degree = 0.0;
    pub overhang_degree: f64,
    // TreeSupport.hpp:133: ExPolygon overhang;
    pub overhang: ExPolygon,
    // TreeSupport.hpp:134: coordf_t origin_area;
    pub origin_area: CoordF,
    // TreeSupport.hpp:135: coordf_t target_radius = -1.;
    pub target_radius: CoordF,

    // TreeSupport.hpp:143: Point skin_direction;
    pub skin_direction: Point,

    // TreeSupport.hpp:153: int support_roof_layers_below;
    pub support_roof_layers_below: i32,
    // TreeSupport.hpp:154: int obj_layer_nr;
    pub obj_layer_nr: i32,

    // TreeSupport.hpp:163: bool to_buildplate;
    pub to_buildplate: bool,

    // TreeSupport.hpp:172: SupportNode* parent;
    // Represented as an arena index (None == NO_PARENT == nullptr).
    pub parent: Option<usize>,
    // TreeSupport.hpp:173: std::vector<SupportNode*> parents;
    pub parents: Vec<usize>,
    // TreeSupport.hpp:174: SupportNode* child = nullptr;
    pub child: Option<usize>,

    // TreeSupport.hpp:183: std::list<SupportNode*> merged_neighbours;
    pub merged_neighbours: Vec<usize>,

    // TreeSupport.hpp:185: coordf_t print_z;
    pub print_z: CoordF,
    // TreeSupport.hpp:186: coordf_t height;
    pub height: CoordF,
}

impl SupportNode {
    // TreeSupport.hpp:51: static constexpr SupportNode* NO_PARENT = nullptr;
    pub const NO_PARENT: Option<usize> = None;

    // TreeSupport.hpp:53-63: SupportNode() default constructor
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

    /// TreeSupport.hpp:66-97: full SupportNode constructor.
    ///
    /// `nodes` is the arena holding all already-created SupportNodes; `parent`
    /// is the arena index of the parent node (or None for NO_PARENT). The
    /// `diameter_angle_scale_factor` is the static member (TreeSupport.hpp:115)
    /// passed in explicitly. Returns the new node; the caller is responsible for
    /// pushing it onto the arena and updating `parent->child` linkage as C++ does
    /// (which it cannot do here because it would mutate `nodes`).
    #[allow(clippy::too_many_arguments)]
    pub fn with_parent(
        nodes: &mut [SupportNode],
        position: Point,
        distance_to_top: i32,
        obj_layer_nr: i32,
        support_roof_layers_below: i32,
        to_buildplate: bool,
        parent: Option<usize>,
        print_z_: CoordF,
        height_: CoordF,
        dist_mm_to_top_: CoordF,
        radius_: CoordF,
        diameter_angle_scale_factor: f64,
        self_idx: usize,
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
            movement: Point::new(0, 0),
            max_move_dist: 0.0,
            type_: TreeNodeType::ECircle,
            is_corner: false,
            is_processed: false,
            need_extra_wall: false,
            is_sharp_tail: false,
            valid: true,
            fading: false,
            overhang_degree: 0.0,
            overhang: ExPolygon::default(),
            origin_area: 0.0,
            target_radius: -1.0,
            skin_direction: Point::new(0, 0),
            parents: Vec::new(),
            child: None,
            merged_neighbours: Vec::new(),
        };

        // TreeSupport.hpp:80-96
        if let Some(parent_idx) = parent {
            // parents.push_back(parent);
            node.parents.push(parent_idx);
            // type = parent->type;
            node.type_ = nodes[parent_idx].type_;
            // overhang = parent->overhang;
            node.overhang = nodes[parent_idx].overhang.clone();
            // orig_pos = parent->orig_pos;
            node.orig_pos = nodes[parent_idx].orig_pos;
            // if (dist_mm_to_top == 0) dist_mm_to_top = parent->dist_mm_to_top + parent->height;
            if node.dist_mm_to_top == 0.0 {
                node.dist_mm_to_top = nodes[parent_idx].dist_mm_to_top + nodes[parent_idx].height;
            }
            // if (radius == 0 && parent->radius>0)
            //     radius = parent->radius + (dist_mm_to_top - parent->dist_mm_to_top) * diameter_angle_scale_factor;
            if node.radius == 0.0 && nodes[parent_idx].radius > 0.0 {
                node.radius = nodes[parent_idx].radius
                    + (node.dist_mm_to_top - nodes[parent_idx].dist_mm_to_top)
                        * diameter_angle_scale_factor;
            }
            // parent->child = this;
            nodes[parent_idx].child = Some(self_idx);
            // for (auto& neighbor : parent->merged_neighbours) { neighbor->child = this; parents.push_back(neighbor); }
            let neighbours = nodes[parent_idx].merged_neighbours.clone();
            for neighbor in neighbours {
                nodes[neighbor].child = Some(self_idx);
                node.parents.push(neighbor);
            }
            // is_sharp_tail = parent->is_sharp_tail;
            node.is_sharp_tail = nodes[parent_idx].is_sharp_tail;
            // skin_direction = parent->skin_direction;
            node.skin_direction = nodes[parent_idx].skin_direction;
        }

        node
    }

    // TreeSupport.hpp:188-191: bool operator==(const SupportNode& other) const { return position == other.position; }
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
    // TreeSupport.hpp:188-191
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

// ============================================================================
// TreeSupport.cpp free functions / static helpers
// ============================================================================

// TreeSupport.cpp:59-62
// inline double dot_with_unscale(const Point a, const Point b)
// { return unscale_(a(0)) * unscale_(b(0)) + unscale_(a(1)) * unscale_(b(1)); }
#[inline]
pub fn dot_with_unscale(a: Point, b: Point) -> f64 {
    unscale_(a.x) * unscale_(b.x) + unscale_(a.y) * unscale_(b.y)
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
    let length = scale_(vsize2_with_unscale(pt).sqrt()) as f64;
    // C++: return pt * (scale / length); — coord_t() truncation (see point_mul_trunc).
    point_mul_trunc(pt, scale_factor / length)
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
            let x = a + point_mul_trunc(ab, dot_prod / ab_length2);

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
                let x = a + point_mul_trunc(ab, dot_prod / ab_length2);

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
    // offset_ex default join type is ClipperLib::jtMiter (ClipperUtils.hpp:31,355).
    // FIDELITY-NOTE(F1): geo-clipper approximation vs C++ ClipperLib.
    let polys_dilated = crate::clipper_utils::union_ex(&crate::clipper_utils::offset_expolygons(
        polygons,
        scale_(distance) as CoordF,
        crate::clipper_utils::OffsetJoinType::Miter,
    ));
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
    let moment = (ixx * crate::SCALING_FACTOR.powi(4)).min(iyy * crate::SCALING_FACTOR.powi(4));
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
    // C++ diff_ex(const ExPolygon&, const Polygons&) (ClipperUtils.hpp:452): re-assemble
    // the clip Polygons into ExPolygons (recovering holes from winding) before differencing.
    let clip_ex = crate::clipper_utils::union_polygons_ex(&clipped_avoid_region);
    let expolys_avoid = crate::clipper_utils::difference(&[expoly.clone()], &clip_ex);
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
