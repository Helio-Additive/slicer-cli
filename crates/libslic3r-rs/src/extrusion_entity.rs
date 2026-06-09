//! Faithful 1:1 port of `src/libslic3r/ExtrusionEntity.{hpp,cpp}` (BambuStudio).
//!
//! C++ Reference:
//! - ExtrusionEntity.hpp
//! - ExtrusionEntity.cpp
//!
//! Extrusion entities represent toolpath segments with associated printing parameters.
//!
//! Representation notes (divergences from C++ documented here, behaviour preserved):
//! - C++ uses an `ExtrusionEntity*` class hierarchy with virtual dispatch. The Rust
//!   port models the concrete leaf types (`ExtrusionPath`, `ExtrusionLoop`,
//!   `ExtrusionMultiPath`, `ExtrusionEntityCollection`) as plain structs plus a shared
//!   `ExtrusionEntity` trait. `ExtrusionEntityCollection`/`ExtrusionEntityType` live in
//!   this module (used by `crate::extrusion_entity_collection`).
//! - C++ `float width; float height;` and `double overhang_degree;` are represented here
//!   as `CoordF`(f64) / `i32` for ergonomics with the rest of the crate; arithmetic is
//!   otherwise identical.
//! - `coord_t` -> `i64`, `coordf_t` -> `f64`.

// ExtrusionEntity.cpp:1-10
// #include "ExtrusionEntity.hpp"
// #include "ExtrusionEntityCollection.hpp"
// #include "ExPolygon.hpp"
// #include "ClipperUtils.hpp"
// #include "Extruder.hpp"
// #include "Flow.hpp"
// #include <cmath>
// #include <limits>
// #include <sstream>
// #include "Utils.hpp"

use crate::clipper_utils::{diff_pl, intersection_pl, offset_polyline};
use crate::flow::Flow;
use crate::geometry::{foot_pt, ExPolygon, Line, Point, Polygon, Polyline, Polylines};
use crate::libslic3r::EPSILON;
use crate::{scale, unscale, CoordF, SCALING_FACTOR};

// ExtrusionEntity.cpp:12
// #define L(s) (s)

// ExtrusionEntity.cpp:14
// namespace Slic3r {
// ExtrusionEntity.cpp:15
// static const double slope_path_ratio = 0.3;
const SLOPE_PATH_RATIO: f64 = 0.3;
// ExtrusionEntity.cpp:16
// static const double slope_inner_outer_wall_gap = 0.4;
const SLOPE_INNER_OUTER_WALL_GAP: f64 = 0.4;
// ExtrusionEntity.cpp:17
// static const int    overhang_threshold = 1;
const OVERHANG_THRESHOLD: i32 = 1;

// ===========================================================================
// ExtrusionEntity.hpp:21-42 — NodeContour / LoopNode
// ===========================================================================

/// ExtrusionEntity.hpp:21-26 `struct NodeContour`
#[derive(Debug, Clone, Default)]
pub struct NodeContour {
    /// for lines contour
    pub pts: Vec<Point>,
    pub widths: Vec<i64>,
    pub is_loop: bool,
}

/// ExtrusionEntity.hpp:28-42 `struct LoopNode`
#[derive(Debug, Clone)]
pub struct LoopNode {
    // store outer wall and mark if it's loop
    pub node_contour: NodeContour,
    pub node_id: i32,
    pub loop_id: i32,
    pub bbox: crate::geometry::BoundingBox,
    pub merged_id: i32,
    // upper loop info
    pub upper_node_id: Vec<i32>,
    // lower loop info
    pub lower_node_id: Vec<i32>,
}

// ===========================================================================
// ExtrusionEntity.hpp:45-70 — ExtrusionRole
// ===========================================================================

/// Each ExtrusionRole value identifies a distinct set of { extruder, speed }
/// ExtrusionEntity.hpp:45-70
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExtrusionRole {
    None,
    Perimeter,
    ExternalPerimeter,
    OverhangPerimeter,
    InternalInfill,
    SolidInfill,
    FloatingVerticalShell,
    TopSolidInfill,
    BottomSurface,
    Ironing,
    BridgeInfill,
    GapFill,
    Skirt,
    Brim,
    SupportMaterial,
    SupportMaterialInterface,
    SupportTransition,
    SupportIroning,
    WipeTower,
    Custom,
    Flush,
    /// Extrusion role for a collection with multiple extrusion roles.
    Mixed,
    // erCount is the sentinel and is not represented as a variant.
}

// ExtrusionEntity.hpp:72-76 `enum CustomizeFlag`
/// Special flags describing loop customization
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CustomizeFlag {
    None,
    CircleCompensation, // shaft hole tolerance compensation
    FloatingVerticalShell,
}

// ExtrusionEntity.hpp:78-85 `enum ExtrusionLoopRole`
/// Special flags describing loop role
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ExtrusionLoopRole(pub u8);

impl ExtrusionLoopRole {
    // ExtrusionEntity.hpp:80 elrDefault = 1 << 0
    pub const DEFAULT: Self = Self(1 << 0);
    // ExtrusionEntity.hpp:81 elrContourInternalPerimeter = 1 << 1
    pub const CONTOUR_INTERNAL_PERIMETER: Self = Self(1 << 1);
    // ExtrusionEntity.hpp:82 elrSkirt = 1 << 2
    pub const SKIRT: Self = Self(1 << 2);
    // ExtrusionEntity.hpp:83 elrPerimeterHole = 1 << 3
    pub const PERIMETER_HOLE: Self = Self(1 << 3);
    // ExtrusionEntity.hpp:84 elrSecondPerimeter = 1 << 4
    pub const SECOND_PERIMETER: Self = Self(1 << 4);

    pub fn new() -> Self {
        Self::DEFAULT
    }

    pub fn contains(&self, other: Self) -> bool {
        (self.0 & other.0) != 0
    }
}

impl Default for ExtrusionLoopRole {
    fn default() -> Self {
        Self::new()
    }
}

// ExtrusionEntity.hpp:87-89 `inline ExtrusionLoopRole operator |(...)`
impl std::ops::BitOr for ExtrusionLoopRole {
    type Output = Self;

    fn bitor(self, rhs: Self) -> Self::Output {
        Self(self.0 | rhs.0)
    }
}

impl std::ops::BitOrAssign for ExtrusionLoopRole {
    fn bitor_assign(&mut self, rhs: Self) {
        self.0 |= rhs.0;
    }
}

// ExtrusionEntity.hpp:92-97 `inline bool is_perimeter(ExtrusionRole role)`
pub fn is_perimeter(role: ExtrusionRole) -> bool {
    role == ExtrusionRole::Perimeter
        || role == ExtrusionRole::ExternalPerimeter
        || role == ExtrusionRole::OverhangPerimeter
}

// ExtrusionEntity.hpp:99-108 `inline bool is_infill(ExtrusionRole role)`
pub fn is_infill(role: ExtrusionRole) -> bool {
    role == ExtrusionRole::BridgeInfill
        || role == ExtrusionRole::InternalInfill
        || role == ExtrusionRole::SolidInfill
        || role == ExtrusionRole::FloatingVerticalShell
        || role == ExtrusionRole::TopSolidInfill
        || role == ExtrusionRole::BottomSurface
        || role == ExtrusionRole::Ironing
}

// ExtrusionEntity.hpp:110-113 `inline bool is_top_surface(ExtrusionRole role)`
pub fn is_top_surface(role: ExtrusionRole) -> bool {
    role == ExtrusionRole::TopSolidInfill
}

// ExtrusionEntity.hpp:115-123 `inline bool is_solid_infill(ExtrusionRole role)`
pub fn is_solid_infill(role: ExtrusionRole) -> bool {
    role == ExtrusionRole::BridgeInfill
        || role == ExtrusionRole::SolidInfill
        || role == ExtrusionRole::FloatingVerticalShell
        || role == ExtrusionRole::TopSolidInfill
        || role == ExtrusionRole::BottomSurface
        || role == ExtrusionRole::Ironing
}

// ExtrusionEntity.hpp:125-128 `inline bool is_bridge(ExtrusionRole role)`
pub fn is_bridge(role: ExtrusionRole) -> bool {
    role == ExtrusionRole::BridgeInfill || role == ExtrusionRole::OverhangPerimeter
}

// ExtrusionEntity.hpp:131-136 `inline bool is_support(ExtrusionRole role)`
pub fn is_support(role: ExtrusionRole) -> bool {
    role == ExtrusionRole::SupportMaterial
        || role == ExtrusionRole::SupportMaterialInterface
        || role == ExtrusionRole::SupportTransition
        || role == ExtrusionRole::SupportIroning
}

// Method-style wrappers, kept for existing call sites; delegate to the free fns above.
impl ExtrusionRole {
    pub fn is_perimeter(&self) -> bool {
        is_perimeter(*self)
    }
    pub fn is_infill(&self) -> bool {
        is_infill(*self)
    }
    pub fn is_top_surface(&self) -> bool {
        is_top_surface(*self)
    }
    pub fn is_solid_infill(&self) -> bool {
        is_solid_infill(*self)
    }
    pub fn is_bridge(&self) -> bool {
        is_bridge(*self)
    }
    pub fn is_support(&self) -> bool {
        is_support(*self)
    }

    /// ExtrusionEntity.cpp:613-641 `ExtrusionEntity::role_to_string`
    pub fn to_string(&self) -> &'static str {
        role_to_string(*self)
    }

    /// ExtrusionEntity.cpp:643-689 `ExtrusionEntity::string_to_role`
    pub fn from_string(s: &str) -> Self {
        string_to_role(s)
    }
}

// ===========================================================================
// ExtrusionEntity.hpp:212-383 — ExtrusionPath
// ===========================================================================

/// A single extrusion path with constant width, height, and role.
/// ExtrusionEntity.hpp:212-383 `class ExtrusionPath : public ExtrusionEntity`
#[derive(Debug, Clone)]
pub struct ExtrusionPath {
    // ExtrusionEntity.hpp:215 `Polyline polyline;`
    pub polyline: Polyline,
    // ExtrusionEntity.hpp:216 `double overhang_degree = 0;`
    pub overhang_degree: i32,
    // ExtrusionEntity.hpp:217 `int curve_degree = 0;`
    pub curve_degree: i32,
    // ExtrusionEntity.hpp:219 `double mm3_per_mm;`
    pub mm3_per_mm: CoordF,
    // ExtrusionEntity.hpp:221 `float width;`
    pub width: CoordF,
    // ExtrusionEntity.hpp:223 `float height;`
    pub height: CoordF,
    // ExtrusionEntity.hpp:224 `double smooth_speed = 0;`
    pub smooth_speed: CoordF,
    // ExtrusionEntity.hpp:379 `bool m_can_reverse = true;`
    pub can_reverse: bool,
    // ExtrusionEntity.hpp:380 `ExtrusionRole m_role;`
    pub role: ExtrusionRole,
    // ExtrusionEntity.hpp:382 `bool m_no_extrusion = false;`
    pub no_extrusion: bool,
    // ExtrusionEntity.hpp:206 `CustomizeFlag m_customize_flag{CustomizeFlag::cfNone};`
    pub customize_flag: CustomizeFlag,
    // ExtrusionEntity.hpp:207 `int m_cooling_node{ -1 };`
    pub cooling_node: i32,
}

impl ExtrusionPath {
    // ExtrusionEntity.hpp:227 `ExtrusionPath(ExtrusionRole role) : mm3_per_mm(-1), width(-1), height(-1), m_role(role), m_no_extrusion(false) {}`
    pub fn new(role: ExtrusionRole) -> Self {
        Self {
            polyline: Polyline::new(),
            overhang_degree: 0,
            curve_degree: 0,
            mm3_per_mm: -1.0,
            width: -1.0,
            height: -1.0,
            smooth_speed: 0.0,
            can_reverse: true,
            role,
            no_extrusion: false,
            customize_flag: CustomizeFlag::None,
            cooling_node: -1,
        }
    }

    // ExtrusionEntity.hpp:228 `ExtrusionPath(ExtrusionRole role, double mm3_per_mm, float width, float height, bool no_extrusion = false)`
    pub fn with_params(
        role: ExtrusionRole,
        mm3_per_mm: CoordF,
        width: CoordF,
        height: CoordF,
        no_extrusion: bool,
    ) -> Self {
        Self {
            polyline: Polyline::new(),
            overhang_degree: 0,
            curve_degree: 0,
            mm3_per_mm,
            width,
            height,
            smooth_speed: 0.0,
            can_reverse: true,
            role,
            no_extrusion,
            customize_flag: CustomizeFlag::None,
            cooling_node: -1,
        }
    }

    // ExtrusionEntity.hpp:229 `ExtrusionPath(double overhang_degree, int curve_degree, ExtrusionRole role, double mm3_per_mm, float width, float height)`
    pub fn with_overhang(
        overhang_degree: i32,
        curve_degree: i32,
        role: ExtrusionRole,
        mm3_per_mm: CoordF,
        width: CoordF,
        height: CoordF,
    ) -> Self {
        Self {
            polyline: Polyline::new(),
            overhang_degree,
            curve_degree,
            mm3_per_mm,
            width,
            height,
            smooth_speed: 0.0,
            can_reverse: true,
            role,
            no_extrusion: false,
            customize_flag: CustomizeFlag::None,
            cooling_node: -1,
        }
    }

    // ExtrusionEntity.hpp:257-269 `ExtrusionPath(const Polyline &polyline, const ExtrusionPath &rhs)`
    /// Construct from a polyline copying all other attributes from `rhs`.
    pub fn from_polyline_and(polyline: Polyline, rhs: &ExtrusionPath) -> Self {
        Self {
            polyline,
            overhang_degree: rhs.overhang_degree,
            curve_degree: rhs.curve_degree,
            mm3_per_mm: rhs.mm3_per_mm,
            width: rhs.width,
            height: rhs.height,
            smooth_speed: rhs.smooth_speed,
            can_reverse: rhs.can_reverse,
            role: rhs.role,
            no_extrusion: rhs.no_extrusion,
            customize_flag: rhs.customize_flag,
            cooling_node: rhs.cooling_node,
        }
    }

    // ExtrusionEntity.cpp:19-22 `void ExtrusionPath::intersect_expolygons(...)`
    pub fn intersect_expolygons(
        &self,
        collection: &[ExPolygon],
        retval: &mut ExtrusionEntityCollection,
    ) {
        // ExtrusionEntity.cpp:21
        self._inflate_collection(
            &intersection_pl(std::slice::from_ref(&self.polyline), collection),
            retval,
        );
    }

    // ExtrusionEntity.cpp:24-27 `void ExtrusionPath::subtract_expolygons(...)`
    pub fn subtract_expolygons(
        &self,
        collection: &[ExPolygon],
        retval: &mut ExtrusionEntityCollection,
    ) {
        // ExtrusionEntity.cpp:26
        self._inflate_collection(
            &diff_pl(std::slice::from_ref(&self.polyline), collection),
            retval,
        );
    }

    // ExtrusionEntity.cpp:29-32 `void ExtrusionPath::clip_end(double distance)`
    pub fn clip_end(&mut self, distance: f64) {
        // ExtrusionEntity.cpp:31
        self.polyline.clip_end(distance);
    }

    // ExtrusionEntity.cpp:34-37 `void ExtrusionPath::simplify(double tolerance)`
    pub fn simplify(&mut self, tolerance: f64) {
        // ExtrusionEntity.cpp:36
        self.polyline.simplify(tolerance);
    }

    // ExtrusionEntity.cpp:39-42 `void ExtrusionPath::simplify_by_fitting_arc(double tolerance)`
    pub fn simplify_by_fitting_arc(&mut self, tolerance: f64) {
        // ExtrusionEntity.cpp:41
        self.polyline.simplify_by_fitting_arc(tolerance);
    }

    // ExtrusionEntity.cpp:44-47 `double ExtrusionPath::length() const`
    pub fn length(&self) -> CoordF {
        // ExtrusionEntity.cpp:46
        self.polyline.length()
    }

    // ExtrusionEntity.cpp:49-53 `void ExtrusionPath::_inflate_collection(...)`
    fn _inflate_collection(&self, polylines: &Polylines, collection: &mut ExtrusionEntityCollection) {
        // ExtrusionEntity.cpp:51-52
        for polyline in polylines {
            collection
                .entities
                .push(ExtrusionEntityType::Path(ExtrusionPath::from_polyline_and(
                    polyline.clone(),
                    self,
                )));
        }
    }

    // ExtrusionEntity.cpp:55-58 `void ExtrusionPath::polygons_covered_by_width(...)`
    pub fn polygons_covered_by_width(&self, out: &mut Vec<Polygon>, scaled_epsilon: f32) {
        // ExtrusionEntity.cpp:57
        let delta = scale(self.width / 2.0) as f32 + scaled_epsilon;
        out.extend(offset_polyline(&self.polyline, delta as f64));
    }

    // ExtrusionEntity.cpp:60-68 `void ExtrusionPath::polygons_covered_by_spacing(...)`
    pub fn polygons_covered_by_spacing(&self, out: &mut Vec<Polygon>, scaled_epsilon: f32) {
        // Instantiating the Flow class to get the line spacing.
        // Don't know the nozzle diameter, setting to zero. It shall not matter it shall be optimized out by the compiler.
        // ExtrusionEntity.cpp:64
        let bridge = is_bridge(self.role);
        // ExtrusionEntity.cpp:65
        debug_assert!(!bridge || self.width == self.height);
        // ExtrusionEntity.cpp:66
        let flow = if bridge {
            Flow::bridging_flow(self.width, 0.0)
        } else {
            Flow::new(self.width, self.height, 0.0).unwrap()
        };
        // ExtrusionEntity.cpp:67
        let delta = 0.5_f32 * flow.scaled_spacing() as f32 + scaled_epsilon;
        out.extend(offset_polyline(&self.polyline, delta as f64));
    }

    // ExtrusionEntity.cpp:70-80 `bool ExtrusionPath::can_merge(const ExtrusionPath& other)`
    pub fn can_merge(&self, other: &ExtrusionPath) -> bool {
        // ExtrusionEntity.cpp:72-79
        self.curve_degree == other.curve_degree
            && self.mm3_per_mm == other.mm3_per_mm
            && self.width == other.width
            && self.height == other.height
            && self.can_reverse == other.can_reverse
            && self.role == other.role
            && self.no_extrusion == other.no_extrusion
            && self.smooth_speed == other.smooth_speed
    }

    // ExtrusionEntity.hpp:316 `void reverse() override { this->polyline.reverse(); }`
    pub fn reverse(&mut self) {
        self.polyline.reverse();
    }

    // ExtrusionEntity.hpp:317 `const Point& first_point() const override`
    pub fn first_point(&self) -> Point {
        self.polyline.points()[0]
    }

    // ExtrusionEntity.hpp:318 `const Point& last_point() const override`
    pub fn last_point(&self) -> Point {
        *self.polyline.points().last().unwrap()
    }

    // ExtrusionEntity.hpp:319 `size_t size() const`
    pub fn size(&self) -> usize {
        self.polyline.size()
    }

    // ExtrusionEntity.hpp:320 `bool empty() const`
    pub fn empty(&self) -> bool {
        self.polyline.empty()
    }

    // ExtrusionEntity.hpp:321 `bool is_closed() const`
    pub fn is_closed(&self) -> bool {
        !self.empty() && self.polyline.points()[0] == *self.polyline.points().last().unwrap()
    }

    // ExtrusionEntity.hpp:331 `ExtrusionRole role() const override { return m_role; }`
    // NOTE: the role is exposed as the public field `role`; the C++ `role()` accessor
    // is provided via the `ExtrusionEntity` trait (no inherent method, to avoid a
    // field/method name clash). Internally we read `self.role` directly.

    // ExtrusionEntity.hpp:344 `double min_mm3_per_mm() const override { return this->mm3_per_mm; }`
    pub fn min_mm3_per_mm(&self) -> CoordF {
        self.mm3_per_mm
    }

    // ExtrusionEntity.hpp:345 `Polyline as_polyline() const override { return this->polyline; }`
    pub fn as_polyline(&self) -> Polyline {
        self.polyline.clone()
    }

    // ExtrusionEntity.hpp:348 `double total_volume() const override { return mm3_per_mm * unscale<double>(length()); }`
    pub fn total_volume(&self) -> CoordF {
        self.mm3_per_mm * (self.length() / SCALING_FACTOR)
    }

    // ExtrusionEntity.hpp:350-353 `void set_overhang_degree(int overhang)`
    pub fn set_overhang_degree(&mut self, overhang: i32) {
        if is_perimeter(self.role) || is_support(self.role) {
            self.overhang_degree = if overhang < 0 {
                0
            } else if overhang > 10 {
                10
            } else {
                overhang
            };
        }
    }

    // ExtrusionEntity.hpp:354-359 `int get_overhang_degree() const`
    pub fn get_overhang_degree(&self) -> i32 {
        // only perimeter has overhang degree. Other return 0;
        if is_perimeter(self.role) || is_support(self.role) {
            return self.overhang_degree;
        }
        0
    }

    // ExtrusionEntity.hpp:360-362 `void set_curve_degree(int curve)`
    pub fn set_curve_degree(&mut self, curve: i32) {
        self.curve_degree = if curve < 0 {
            0
        } else if curve > 10 {
            10
        } else {
            curve
        };
    }

    // ExtrusionEntity.hpp:363-365 `int get_curve_degree() const`
    pub fn get_curve_degree(&self) -> i32 {
        self.curve_degree
    }

    // ExtrusionEntity.hpp:369 `bool is_force_no_extrusion() const`
    pub fn is_force_no_extrusion(&self) -> bool {
        self.no_extrusion
    }

    // ExtrusionEntity.hpp:370 `void set_force_no_extrusion(bool no_extrusion)`
    pub fn set_force_no_extrusion(&mut self, no_extrusion: bool) {
        self.no_extrusion = no_extrusion;
    }

    // ExtrusionEntity.hpp:371 `void set_extrusion_role(ExtrusionRole extrusion_role)`
    pub fn set_extrusion_role(&mut self, extrusion_role: ExtrusionRole) {
        self.role = extrusion_role;
    }

    // ExtrusionEntity.hpp:372 `void set_reverse() override { m_can_reverse = false; }`
    pub fn set_reverse(&mut self) {
        self.can_reverse = false;
    }

    // ExtrusionEntity.hpp:373 `bool can_reverse() const override { return m_can_reverse; }`
    pub fn can_reverse(&self) -> bool {
        self.can_reverse
    }

    // ExtrusionEntity.hpp:199-200 `get/set_customize_flag`
    pub fn set_customize_flag(&mut self, flag: CustomizeFlag) {
        self.customize_flag = flag;
    }
    pub fn get_customize_flag(&self) -> CustomizeFlag {
        self.customize_flag
    }

    // ExtrusionEntity.hpp:202-203 `get/set_cooling_node`
    pub fn get_cooling_node(&self) -> i32 {
        self.cooling_node
    }
    pub fn set_cooling_node(&mut self, id: i32) {
        self.cooling_node = id;
    }
}

// ===========================================================================
// ExtrusionEntity.hpp:385-413 — ExtrusionPathSloped::Slope (data only)
// ===========================================================================

/// ExtrusionEntity.hpp:388-393 `struct Slope`
#[derive(Debug, Clone, Copy)]
pub struct Slope {
    // ExtrusionEntity.hpp:390 `double z_ratio{1.};`
    pub z_ratio: f64,
    // ExtrusionEntity.hpp:391 `double e_ratio{1.};`
    pub e_ratio: f64,
    // ExtrusionEntity.hpp:392 `double speed_record{0.0};`
    pub speed_record: f64,
}

impl Default for Slope {
    fn default() -> Self {
        Self {
            z_ratio: 1.0,
            e_ratio: 1.0,
            speed_record: 0.0,
        }
    }
}

/// ExtrusionEntity.hpp:385-413 `class ExtrusionPathSloped : public ExtrusionPath`
#[derive(Debug, Clone)]
pub struct ExtrusionPathSloped {
    pub path: ExtrusionPath,
    // ExtrusionEntity.hpp:395 `Slope slope_begin;`
    pub slope_begin: Slope,
    // ExtrusionEntity.hpp:396 `Slope slope_end;`
    pub slope_end: Slope,
}

impl ExtrusionPathSloped {
    // ExtrusionEntity.hpp:399-400 `ExtrusionPathSloped(const Polyline &polyline, const ExtrusionPath &rhs, const Slope &begin, const Slope &end)`
    pub fn new(polyline: Polyline, rhs: &ExtrusionPath, begin: Slope, end: Slope) -> Self {
        Self {
            path: ExtrusionPath::from_polyline_and(polyline, rhs),
            slope_begin: begin,
            slope_end: end,
        }
    }

    // ExtrusionEntity.hpp:404-410 `Slope interpolate(const double ratio) const`
    pub fn interpolate(&self, ratio: f64) -> Slope {
        Slope {
            z_ratio: lerp(self.slope_begin.z_ratio, self.slope_end.z_ratio, ratio),
            e_ratio: lerp(self.slope_begin.e_ratio, self.slope_end.e_ratio, ratio),
            speed_record: lerp(
                self.slope_begin.speed_record,
                self.slope_end.speed_record,
                ratio,
            ),
        }
    }

    // ExtrusionEntity.hpp:412 `bool is_flat() const { return is_approx(slope_begin.z_ratio, slope_end.z_ratio); }`
    pub fn is_flat(&self) -> bool {
        is_approx(self.slope_begin.z_ratio, self.slope_end.z_ratio)
    }

    /// Convenience: total length of the underlying path.
    pub fn length(&self) -> CoordF {
        self.path.length()
    }
}

// ===========================================================================
// ExtrusionEntity.hpp:428-490 — ExtrusionMultiPath
// ===========================================================================

/// Single continuous extrusion path, possibly with varying extrusion thickness, extrusion height or bridging / non bridging.
/// ExtrusionEntity.hpp:428-490 `class ExtrusionMultiPath : public ExtrusionEntity`
#[derive(Debug, Clone, Default)]
pub struct ExtrusionMultiPath {
    // ExtrusionEntity.hpp:431 `ExtrusionPaths paths;`
    pub paths: Vec<ExtrusionPath>,
    // ExtrusionEntity.hpp:489 `bool m_can_reverse = true;`
    pub can_reverse: bool,
}

impl ExtrusionMultiPath {
    // ExtrusionEntity.hpp:433 `ExtrusionMultiPath() {}`
    pub fn new() -> Self {
        Self {
            paths: Vec::new(),
            can_reverse: true,
        }
    }

    // ExtrusionEntity.hpp:437 `ExtrusionMultiPath(const ExtrusionPath &path)`
    pub fn from_path(path: ExtrusionPath) -> Self {
        let can_reverse = path.can_reverse();
        Self {
            paths: vec![path],
            can_reverse,
        }
    }

    // ExtrusionEntity.cpp:82-87 `void ExtrusionMultiPath::reverse()`
    pub fn reverse(&mut self) {
        // ExtrusionEntity.cpp:84-85
        for path in &mut self.paths {
            path.reverse();
        }
        // ExtrusionEntity.cpp:86
        self.paths.reverse();
    }

    // ExtrusionEntity.cpp:89-95 `double ExtrusionMultiPath::length() const`
    pub fn length(&self) -> CoordF {
        // ExtrusionEntity.cpp:91
        let mut len = 0.0;
        // ExtrusionEntity.cpp:92-93
        for path in &self.paths {
            len += path.polyline.length();
        }
        // ExtrusionEntity.cpp:94
        len
    }

    // ExtrusionEntity.cpp:97-101 `void ExtrusionMultiPath::polygons_covered_by_width(...)`
    pub fn polygons_covered_by_width(&self, out: &mut Vec<Polygon>, scaled_epsilon: f32) {
        // ExtrusionEntity.cpp:99-100
        for path in &self.paths {
            path.polygons_covered_by_width(out, scaled_epsilon);
        }
    }

    // ExtrusionEntity.cpp:103-107 `void ExtrusionMultiPath::polygons_covered_by_spacing(...)`
    pub fn polygons_covered_by_spacing(&self, out: &mut Vec<Polygon>, scaled_epsilon: f32) {
        // ExtrusionEntity.cpp:105-106
        for path in &self.paths {
            path.polygons_covered_by_spacing(out, scaled_epsilon);
        }
    }

    // ExtrusionEntity.cpp:109-115 `double ExtrusionMultiPath::min_mm3_per_mm() const`
    pub fn min_mm3_per_mm(&self) -> CoordF {
        // ExtrusionEntity.cpp:111
        let mut min_mm3_per_mm = f64::MAX;
        // ExtrusionEntity.cpp:112-113
        for path in &self.paths {
            min_mm3_per_mm = min_mm3_per_mm.min(path.mm3_per_mm);
        }
        // ExtrusionEntity.cpp:114
        min_mm3_per_mm
    }

    // ExtrusionEntity.cpp:117-136 `Polyline ExtrusionMultiPath::as_polyline() const`
    pub fn as_polyline(&self) -> Polyline {
        // ExtrusionEntity.cpp:119
        let mut out = Polyline::new();
        // ExtrusionEntity.cpp:120
        if !self.paths.is_empty() {
            // ExtrusionEntity.cpp:121
            let mut len: usize = 0;
            // ExtrusionEntity.cpp:122-126
            for i_path in 0..self.paths.len() {
                debug_assert!(!self.paths[i_path].polyline.points().is_empty());
                debug_assert!(
                    i_path == 0
                        || *self.paths[i_path - 1].polyline.points().last().unwrap()
                            == self.paths[i_path].polyline.points()[0]
                );
                len += self.paths[i_path].polyline.points().len();
            }
            // The connecting points between the segments are equal.
            // ExtrusionEntity.cpp:128
            len -= self.paths.len() - 1;
            // ExtrusionEntity.cpp:129
            debug_assert!(len > 0);
            // ExtrusionEntity.cpp:130
            out.points_mut().reserve(len);
            // ExtrusionEntity.cpp:131
            out.points_mut()
                .push(self.paths[0].polyline.points()[0]);
            // ExtrusionEntity.cpp:132-133
            for i_path in 0..self.paths.len() {
                out.points_mut()
                    .extend_from_slice(&self.paths[i_path].polyline.points()[1..]);
            }
        }
        // ExtrusionEntity.cpp:135
        out
    }

    // ExtrusionEntity.hpp:459 `const Point& first_point() const override`
    pub fn first_point(&self) -> Point {
        self.paths[0].polyline.points()[0]
    }

    // ExtrusionEntity.hpp:460 `const Point& last_point() const override`
    pub fn last_point(&self) -> Point {
        *self.paths.last().unwrap().polyline.points().last().unwrap()
    }

    // ExtrusionEntity.hpp:461 `size_t size() const`
    pub fn size(&self) -> usize {
        self.paths.len()
    }

    // ExtrusionEntity.hpp:462 `bool empty() const`
    pub fn empty(&self) -> bool {
        self.paths.is_empty()
    }

    // ExtrusionEntity.hpp:464 `ExtrusionRole role() const override { return this->paths.empty() ? erNone : this->paths.front().role(); }`
    pub fn role(&self) -> ExtrusionRole {
        if self.paths.is_empty() {
            ExtrusionRole::None
        } else {
            self.paths[0].role
        }
    }

    // ExtrusionEntity.hpp:453 `bool can_reverse() const override { return m_can_reverse; }`
    pub fn can_reverse(&self) -> bool {
        self.can_reverse
    }

    // ExtrusionEntity.hpp:454 `void set_reverse() override { m_can_reverse = false; }`
    pub fn set_reverse(&mut self) {
        self.can_reverse = false;
    }

    // ExtrusionEntity.hpp:486 `double total_volume() const override`
    pub fn total_volume(&self) -> CoordF {
        let mut volume = 0.0;
        for path in &self.paths {
            volume += path.total_volume();
        }
        volume
    }
}

// ===========================================================================
// ExtrusionEntity.hpp:493-575 — ExtrusionLoop
// ===========================================================================

/// Single continuous extrusion loop, possibly with varying extrusion thickness, extrusion height or bridging / non bridging.
/// ExtrusionEntity.hpp:493-575 `class ExtrusionLoop : public ExtrusionEntity`
#[derive(Debug, Clone)]
pub struct ExtrusionLoop {
    // ExtrusionEntity.hpp:496 `ExtrusionPaths paths;`
    pub paths: Vec<ExtrusionPath>,
    // ExtrusionEntity.hpp:574 `ExtrusionLoopRole m_loop_role;`
    pub loop_role: ExtrusionLoopRole,
    // ExtrusionEntity.hpp:206 `CustomizeFlag m_customize_flag`
    pub customize_flag: CustomizeFlag,
}

/// ExtrusionEntity.hpp:523-528 `struct ClosestPathPoint`
#[derive(Debug, Clone, Copy)]
pub struct ClosestPathPoint {
    pub path_idx: usize,
    pub segment_idx: usize,
    pub foot_pt: Point,
}

impl ExtrusionLoop {
    // ExtrusionEntity.hpp:499 `ExtrusionLoop(const ExtrusionPaths &paths, ExtrusionLoopRole role = elrDefault)`
    pub fn new(paths: Vec<ExtrusionPath>, role: ExtrusionLoopRole) -> Self {
        Self {
            paths,
            loop_role: role,
            customize_flag: CustomizeFlag::None,
        }
    }

    // ExtrusionEntity.hpp:501 `ExtrusionLoop(ExtrusionPaths &&paths, ExtrusionLoopRole role, CustomizeFlag flag)`
    pub fn new_with_flag(
        paths: Vec<ExtrusionPath>,
        role: ExtrusionLoopRole,
        flag: CustomizeFlag,
    ) -> Self {
        Self {
            paths,
            loop_role: role,
            customize_flag: flag,
        }
    }

    // ExtrusionEntity.hpp:502-503 `ExtrusionLoop(const ExtrusionPath &path, ExtrusionLoopRole role = elrDefault)`
    pub fn from_path(path: ExtrusionPath, role: ExtrusionLoopRole) -> Self {
        Self {
            paths: vec![path],
            loop_role: role,
            customize_flag: CustomizeFlag::None,
        }
    }

    // ExtrusionEntity.cpp:138-143 `bool ExtrusionLoop::make_clockwise()`
    pub fn make_clockwise(&mut self) -> bool {
        // ExtrusionEntity.cpp:140
        let was_ccw = self.polygon().is_counter_clockwise();
        // ExtrusionEntity.cpp:141
        if was_ccw {
            self.reverse();
        }
        // ExtrusionEntity.cpp:142
        was_ccw
    }

    // ExtrusionEntity.cpp:145-150 `bool ExtrusionLoop::make_counter_clockwise()`
    pub fn make_counter_clockwise(&mut self) -> bool {
        // ExtrusionEntity.cpp:147
        let was_cw = self.polygon().is_clockwise();
        // ExtrusionEntity.cpp:148
        if was_cw {
            self.reverse();
        }
        // ExtrusionEntity.cpp:149
        was_cw
    }

    // ExtrusionEntity.hpp:513 `bool is_clockwise() { return this->polygon().is_clockwise(); }`
    pub fn is_clockwise(&self) -> bool {
        self.polygon().is_clockwise()
    }

    // ExtrusionEntity.hpp:514 `bool is_counter_clockwise() { return this->polygon().is_counter_clockwise(); }`
    pub fn is_counter_clockwise(&self) -> bool {
        self.polygon().is_counter_clockwise()
    }

    // ExtrusionEntity.cpp:152-157 `void ExtrusionLoop::reverse()`
    pub fn reverse(&mut self) {
        // ExtrusionEntity.cpp:154-155
        for path in &mut self.paths {
            path.reverse();
        }
        // ExtrusionEntity.cpp:156
        self.paths.reverse();
    }

    // ExtrusionEntity.cpp:159-167 `Polygon ExtrusionLoop::polygon() const`
    pub fn polygon(&self) -> Polygon {
        // ExtrusionEntity.cpp:161
        let mut polygon = Polygon::new();
        // ExtrusionEntity.cpp:162-165
        for path in &self.paths {
            // for each polyline, append all points except the last one (because it coincides with the first one of the next polyline)
            let pts = path.polyline.points();
            polygon
                .points_mut()
                .extend_from_slice(&pts[..pts.len() - 1]);
        }
        // ExtrusionEntity.cpp:166
        polygon
    }

    /// Back-compat alias for [`ExtrusionLoop::polygon`] (existing call sites use `as_polygon`).
    pub fn as_polygon(&self) -> Polygon {
        self.polygon()
    }

    // ExtrusionEntity.cpp:169-175 `double ExtrusionLoop::length() const`
    pub fn length(&self) -> CoordF {
        // ExtrusionEntity.cpp:171
        let mut len = 0.0;
        // ExtrusionEntity.cpp:172-173
        for path in &self.paths {
            len += path.polyline.length();
        }
        // ExtrusionEntity.cpp:174
        len
    }

    // ExtrusionEntity.cpp:177-223 `bool ExtrusionLoop::split_at_vertex(const Point &point, const double scaled_epsilon)`
    pub fn split_at_vertex(&mut self, point: &Point, scaled_epsilon: f64) -> bool {
        // ExtrusionEntity.cpp:179-180
        for path_idx in 0..self.paths.len() {
            let idx =
                crate::multi_point::find_point_eps(self.paths[path_idx].polyline.points(), point, scaled_epsilon);
            if idx != -1 {
                let idx = idx as usize;
                // ExtrusionEntity.cpp:181
                if self.paths.len() == 1 {
                    // just change the order of points
                    // ExtrusionEntity.cpp:183-189
                    let mut p1 = Polyline::new();
                    let mut p2 = Polyline::new();
                    self.paths[path_idx]
                        .polyline
                        .split_at_index(idx, &mut p1, &mut p2);
                    if p1.is_valid() && p2.is_valid() {
                        p2.append(&p1);
                        std::mem::swap(self.paths[path_idx].polyline.points_mut(), p2.points_mut());
                        std::mem::swap(
                            &mut self.paths[path_idx].polyline.fitting_result,
                            &mut p2.fitting_result,
                        );
                    }
                } else {
                    // new paths list starts with the second half of current path
                    // ExtrusionEntity.cpp:192-194
                    let mut new_paths: Vec<ExtrusionPath> = Vec::new();
                    let mut p1 = Polyline::new();
                    let mut p2 = Polyline::new();
                    self.paths[path_idx]
                        .polyline
                        .split_at_index(idx, &mut p1, &mut p2);
                    // ExtrusionEntity.cpp:195
                    new_paths.reserve(self.paths.len() + 1);
                    // ExtrusionEntity.cpp:196-201
                    {
                        let mut p = self.paths[path_idx].clone();
                        std::mem::swap(p.polyline.points_mut(), p2.points_mut());
                        std::mem::swap(&mut p.polyline.fitting_result, &mut p2.fitting_result);
                        if p.polyline.is_valid() {
                            new_paths.push(p);
                        }
                    }

                    // then we add all paths until the end of current path list
                    // ExtrusionEntity.cpp:204 — new_paths.insert(end, path+1, this->paths.end()) // not including this path
                    new_paths.extend_from_slice(&self.paths[path_idx + 1..]);

                    // then we add all paths since the beginning of current list up to the previous one
                    // ExtrusionEntity.cpp:207 — new_paths.insert(end, this->paths.begin(), path) // not including this path
                    new_paths.extend_from_slice(&self.paths[..path_idx]);

                    // finally we add the first half of current path
                    // ExtrusionEntity.cpp:210-215
                    {
                        let mut p = self.paths[path_idx].clone();
                        std::mem::swap(p.polyline.points_mut(), p1.points_mut());
                        std::mem::swap(&mut p.polyline.fitting_result, &mut p1.fitting_result);
                        if p.polyline.is_valid() {
                            new_paths.push(p);
                        }
                    }
                    // we can now override the old path list with the new one and stop looping
                    // ExtrusionEntity.cpp:217
                    std::mem::swap(&mut self.paths, &mut new_paths);
                }
                // ExtrusionEntity.cpp:219
                return true;
            }
        }
        // ExtrusionEntity.cpp:222
        false
    }

    // ExtrusionEntity.cpp:225-252 `ExtrusionLoop::ClosestPathPoint ExtrusionLoop::get_closest_path_and_point(...)`
    pub fn get_closest_path_and_point(
        &self,
        point: &Point,
        prefer_non_overhang: bool,
    ) -> ClosestPathPoint {
        // Find the closest path and closest point belonging to that path. Avoid overhangs, if asked for.
        // ExtrusionEntity.cpp:228
        let mut out = ClosestPathPoint {
            path_idx: 0,
            segment_idx: 0,
            foot_pt: Point::new(0, 0),
        };
        // ExtrusionEntity.cpp:229
        let mut min2 = f64::MAX;
        // ExtrusionEntity.cpp:230
        let mut best_non_overhang = ClosestPathPoint {
            path_idx: 0,
            segment_idx: 0,
            foot_pt: Point::new(0, 0),
        };
        // ExtrusionEntity.cpp:231
        let mut min2_non_overhang = f64::MAX;
        // ExtrusionEntity.cpp:232
        for (path_index, path) in self.paths.iter().enumerate() {
            // ExtrusionEntity.cpp:233 — std::pair<int, Point> foot_pt_ = foot_pt(path.polyline.points, point);
            let foot_pt_ = foot_pt(path.polyline.points(), point);
            // ExtrusionEntity.cpp:234 — d2 = (foot_pt_.second - point).cast<double>().squaredNorm();
            let dx = (foot_pt_.1.x - point.x) as f64;
            let dy = (foot_pt_.1.y - point.y) as f64;
            let d2 = dx * dx + dy * dy;
            // ExtrusionEntity.cpp:235-240
            if d2 < min2 {
                out.foot_pt = foot_pt_.1;
                out.path_idx = path_index;
                out.segment_idx = foot_pt_.0 as usize;
                min2 = d2;
            }
            // ExtrusionEntity.cpp:241-246
            if prefer_non_overhang && !is_bridge(path.role) && d2 < min2_non_overhang {
                best_non_overhang.foot_pt = foot_pt_.1;
                best_non_overhang.path_idx = path_index;
                best_non_overhang.segment_idx = foot_pt_.0 as usize;
                min2_non_overhang = d2;
            }
        }
        // ExtrusionEntity.cpp:248-250
        if prefer_non_overhang && min2_non_overhang != f64::MAX {
            // Only apply the non-overhang point if there is one.
            out = best_non_overhang;
        }
        // ExtrusionEntity.cpp:251
        out
    }

    // ExtrusionEntity.cpp:254-306 `void ExtrusionLoop::split_at(const Point &point, bool prefer_non_overhang, const double scaled_epsilon)`
    // Splitting an extrusion loop, possibly made of multiple segments, some of the segments may be bridging.
    pub fn split_at(&mut self, point: &Point, prefer_non_overhang: bool, scaled_epsilon: f64) {
        // ExtrusionEntity.cpp:257-258
        if self.paths.is_empty() {
            return;
        }

        // ExtrusionEntity.cpp:260
        let cpp = self.get_closest_path_and_point(point, prefer_non_overhang);
        let path_idx = cpp.path_idx;
        let segment_idx = cpp.segment_idx;
        let mut p = cpp.foot_pt;

        // Snap p to start or end of segment_idx if closer than scaled_epsilon.
        // ExtrusionEntity.cpp:262-275
        {
            let p1 = self.paths[path_idx].polyline.points()[segment_idx];
            let p2 = self.paths[path_idx].polyline.points()[segment_idx + 1];
            // ExtrusionEntity.cpp:267 — d2_1 = (point - p1).cast<double>().squaredNorm();
            let d2_1 = {
                let dx = (point.x - p1.x) as f64;
                let dy = (point.y - p1.y) as f64;
                dx * dx + dy * dy
            };
            // ExtrusionEntity.cpp:268 — d2_2 = (point - p2).cast<double>().squaredNorm();
            let d2_2 = {
                let dx = (point.x - p2.x) as f64;
                let dy = (point.y - p2.y) as f64;
                dx * dx + dy * dy
            };
            // ExtrusionEntity.cpp:269
            let thr2 = scaled_epsilon * scaled_epsilon;
            // ExtrusionEntity.cpp:270-274
            if d2_1 < d2_2 {
                if d2_1 < thr2 {
                    p = p1;
                }
            } else if d2_2 < thr2 {
                p = p2;
            }
        }

        // now split path_idx in two parts
        // ExtrusionEntity.cpp:278-280
        let (overhang, curve, role, mm3, w, h) = {
            let path = &self.paths[path_idx];
            (
                path.overhang_degree,
                path.curve_degree,
                path.role,
                path.mm3_per_mm,
                path.width,
                path.height,
            )
        };
        let mut p1 = ExtrusionPath::with_overhang(overhang, curve, role, mm3, w, h);
        let mut p2 = ExtrusionPath::with_overhang(overhang, curve, role, mm3, w, h);
        // ExtrusionEntity.cpp:281
        self.paths[path_idx]
            .polyline
            .split_at_point(&mut p, &mut p1.polyline, &mut p2.polyline);

        // ExtrusionEntity.cpp:283
        if self.paths.len() == 1 {
            // ExtrusionEntity.cpp:284-287
            if !p1.polyline.is_valid() {
                std::mem::swap(self.paths[0].polyline.points_mut(), p2.polyline.points_mut());
                std::mem::swap(
                    &mut self.paths[0].polyline.fitting_result,
                    &mut p2.polyline.fitting_result,
                );
            }
            // ExtrusionEntity.cpp:288-291
            else if !p2.polyline.is_valid() {
                std::mem::swap(self.paths[0].polyline.points_mut(), p1.polyline.points_mut());
                std::mem::swap(
                    &mut self.paths[0].polyline.fitting_result,
                    &mut p1.polyline.fitting_result,
                );
            }
            // ExtrusionEntity.cpp:292-296
            else {
                p2.polyline.append(&p1.polyline);
                std::mem::swap(self.paths[0].polyline.points_mut(), p2.polyline.points_mut());
                std::mem::swap(
                    &mut self.paths[0].polyline.fitting_result,
                    &mut p2.polyline.fitting_result,
                );
            }
        } else {
            // install the two paths
            // ExtrusionEntity.cpp:299
            self.paths.remove(path_idx);
            // ExtrusionEntity.cpp:300
            if p2.polyline.is_valid() {
                self.paths.insert(path_idx, p2);
            }
            // ExtrusionEntity.cpp:301
            if p1.polyline.is_valid() {
                self.paths.insert(path_idx, p1);
            }
        }

        // split at the new vertex
        // ExtrusionEntity.cpp:305 — this->split_at_vertex(p); (default scaled_epsilon = scaled<double>(0.001))
        self.split_at_vertex(&p, scale(0.001) as f64);
    }

    // ExtrusionEntity.cpp:308-323 `void ExtrusionLoop::clip_end(double distance, ExtrusionPaths* paths) const`
    pub fn clip_end(&self, mut distance: f64, paths: &mut Vec<ExtrusionPath>) {
        // ExtrusionEntity.cpp:310
        *paths = self.paths.clone();

        // ExtrusionEntity.cpp:312
        while distance > 0.0 && !paths.is_empty() {
            // ExtrusionEntity.cpp:313
            let len = paths.last().unwrap().length();
            // ExtrusionEntity.cpp:315-321
            if len <= distance {
                paths.pop();
                distance -= len;
            } else {
                paths.last_mut().unwrap().polyline.clip_end(distance);
                break;
            }
        }
    }

    // ExtrusionEntity.cpp:408-419 `bool ExtrusionLoop::has_overhang_point(const Point &point) const`
    pub fn has_overhang_point(&self, point: &Point) -> bool {
        // ExtrusionEntity.cpp:410
        for path in &self.paths {
            // ExtrusionEntity.cpp:411
            let pos = path.polyline.find_point(point);
            // ExtrusionEntity.cpp:412
            if pos != -1 {
                // point belongs to this path
                // we consider it overhang only if it's not an endpoint
                // ExtrusionEntity.cpp:415
                return is_bridge(path.role)
                    && pos > 0
                    && pos != (path.polyline.points().len() as i32) - 1;
            }
        }
        // ExtrusionEntity.cpp:418
        false
    }

    // ExtrusionEntity.cpp:421-430 `bool ExtrusionLoop::has_overhang_paths() const`
    pub fn has_overhang_paths(&self) -> bool {
        // ExtrusionEntity.cpp:423
        for path in &self.paths {
            // ExtrusionEntity.cpp:424-425
            if is_bridge(path.role) {
                return true;
            }
            // ExtrusionEntity.cpp:426-427
            if path.overhang_degree >= OVERHANG_THRESHOLD {
                return true;
            }
        }
        // ExtrusionEntity.cpp:429
        false
    }

    // ExtrusionEntity.cpp:432-436 `void ExtrusionLoop::polygons_covered_by_width(...)`
    pub fn polygons_covered_by_width(&self, out: &mut Vec<Polygon>, scaled_epsilon: f32) {
        // ExtrusionEntity.cpp:434-435
        for path in &self.paths {
            path.polygons_covered_by_width(out, scaled_epsilon);
        }
    }

    // ExtrusionEntity.cpp:438-442 `void ExtrusionLoop::polygons_covered_by_spacing(...)`
    pub fn polygons_covered_by_spacing(&self, out: &mut Vec<Polygon>, scaled_epsilon: f32) {
        // ExtrusionEntity.cpp:440-441
        for path in &self.paths {
            path.polygons_covered_by_spacing(out, scaled_epsilon);
        }
    }

    // ExtrusionEntity.cpp:444-450 `double ExtrusionLoop::min_mm3_per_mm() const`
    pub fn min_mm3_per_mm(&self) -> CoordF {
        // ExtrusionEntity.cpp:446
        let mut min_mm3_per_mm = f64::MAX;
        // ExtrusionEntity.cpp:447-448
        for path in &self.paths {
            min_mm3_per_mm = min_mm3_per_mm.min(path.mm3_per_mm);
        }
        // ExtrusionEntity.cpp:449
        min_mm3_per_mm
    }

    // ExtrusionEntity.cpp:452-512 `bool ExtrusionLoop::check_seam_point_angle(double angle_threshold, double min_arm_length) const`
    // Orca: This function is used to check if the loop is smooth(continuous) or not.
    // BBS: only check angle of seam point while the seam has been decided.
    pub fn check_seam_point_angle(&self, angle_threshold: f64, min_arm_length: f64) -> bool {
        // go through all the points in the loop and check if the angle between two segments(AB and BC) is less than the threshold
        // ExtrusionEntity.cpp:457
        let mut idx_prev: usize = 0;
        // ExtrusionEntity.cpp:458
        let idx_curr: usize = 0;
        // ExtrusionEntity.cpp:459
        let mut idx_next: usize = 0;

        // ExtrusionEntity.cpp:461
        let mut distance_to_prev: f32 = 0.0;
        // ExtrusionEntity.cpp:462
        let mut distance_to_next: f32 = 0.0;

        // ExtrusionEntity.cpp:464
        let _polygon = self.polygon();
        // ExtrusionEntity.cpp:465
        let points = _polygon.points();

        // ExtrusionEntity.cpp:467
        let mut lengths: Vec<f32> = Vec::new();
        // ExtrusionEntity.cpp:468 — for each adjacent pair, (unscale(p[i]) - unscale(p[i+1])).norm()
        for point_idx in 0..points.len() - 1 {
            lengths.push(unscale_point_norm(&points[point_idx], &points[point_idx + 1]) as f32);
        }
        // ExtrusionEntity.cpp:469 — std::max((unscale(p[0]) - unscale(p.back())).norm(), 0.1)
        lengths.push(unscale_point_norm(&points[0], &points[points.len() - 1]).max(0.1) as f32);

        // push idx_prev far enough back as initialization
        // ExtrusionEntity.cpp:472-475
        while distance_to_prev < min_arm_length as f32 {
            idx_prev = crate::utils::prev_idx_modulo(idx_prev, points.len());
            distance_to_prev += lengths[idx_prev];
        }

        // push idx_next forward as far as needed
        // ExtrusionEntity.cpp:478-481
        while distance_to_next < min_arm_length as f32 {
            distance_to_next += lengths[idx_next];
            idx_next = crate::utils::next_idx_modulo(idx_next, points.len());
        }

        // thanks orca
        // ExtrusionEntity.cpp:484
        let mut idx_curr = idx_curr;
        for _i in 0..points.len() {
            // pull idx_prev to current as much as possible, while respecting the min_arm_length
            // ExtrusionEntity.cpp:486-489
            while distance_to_prev - lengths[idx_prev] > min_arm_length as f32 {
                distance_to_prev -= lengths[idx_prev];
                idx_prev = crate::utils::next_idx_modulo(idx_prev, points.len());
            }

            // push idx_next forward as far as needed
            // ExtrusionEntity.cpp:492-495
            while distance_to_next < min_arm_length as f32 {
                distance_to_next += lengths[idx_next];
                idx_next = crate::utils::next_idx_modulo(idx_next, points.len());
            }

            // Calculate angle between idx_prev, idx_curr, idx_next.
            // ExtrusionEntity.cpp:498-500
            let p0 = points[idx_prev];
            let p1 = points[idx_curr];
            let p2 = points[idx_next];
            // ExtrusionEntity.cpp:501 — a = angle(p0 - p1, p2 - p1);
            let a = angle(p0 - p1, p2 - p1);
            // ExtrusionEntity.cpp:502
            if if a > 0.0 {
                a < angle_threshold
            } else {
                a > -angle_threshold
            } {
                return false;
            }

            // increase idx_curr by one
            // ExtrusionEntity.cpp:505-508
            let curr_distance = lengths[idx_curr];
            idx_curr += 1;
            distance_to_prev += curr_distance;
            distance_to_next -= curr_distance;
        }

        // ExtrusionEntity.cpp:511
        true
    }

    // ExtrusionEntity.hpp:516 `bool is_set_speed_discontinuity_area() const`
    pub fn is_set_speed_discontinuity_area(&self) -> bool {
        self.role() == ExtrusionRole::ExternalPerimeter
            || self.role() == ExtrusionRole::Perimeter
            || self.role() == ExtrusionRole::OverhangPerimeter
    }

    // ExtrusionEntity.hpp:517 `const Point& first_point() const override`
    pub fn first_point(&self) -> Point {
        self.paths[0].polyline.points()[0]
    }

    // ExtrusionEntity.hpp:518 `const Point& last_point() const override`
    pub fn last_point(&self) -> Point {
        debug_assert!(
            self.first_point() == *self.paths.last().unwrap().polyline.points().last().unwrap()
        );
        self.first_point()
    }

    // ExtrusionEntity.hpp:535 `ExtrusionRole role() const override { return this->paths.empty() ? erNone : this->paths.front().role(); }`
    pub fn role(&self) -> ExtrusionRole {
        if self.paths.is_empty() {
            ExtrusionRole::None
        } else {
            self.paths[0].role
        }
    }

    // ExtrusionEntity.hpp:536 `ExtrusionLoopRole loop_role() const { return m_loop_role; }`
    pub fn loop_role(&self) -> ExtrusionLoopRole {
        self.loop_role
    }

    // ExtrusionEntity.hpp:537 `void set_loop_role(ExtrusionLoopRole role)`
    pub fn set_loop_role(&mut self, role: ExtrusionLoopRole) {
        self.loop_role = role;
    }

    // ExtrusionEntity.hpp:551 `Polyline as_polyline() const override { return this->polygon().split_at_first_point(); }`
    pub fn as_polyline(&self) -> Polyline {
        self.polygon().split_at_first_point()
    }

    // ExtrusionEntity.hpp:559 `double total_volume() const override`
    pub fn total_volume(&self) -> CoordF {
        let mut volume = 0.0;
        for path in &self.paths {
            volume += path.total_volume();
        }
        volume
    }

    /// Sets the customize flag on the loop and all contained paths.
    pub fn set_customize_flag(&mut self, flag: CustomizeFlag) {
        self.customize_flag = flag;
        for path in &mut self.paths {
            path.set_customize_flag(flag);
        }
    }
}

// ===========================================================================
// ExtrusionEntity.hpp:577-593 / ExtrusionEntity.cpp:325-611 — ExtrusionLoopSloped
// ===========================================================================

/// ExtrusionEntity.hpp:577-593 `class ExtrusionLoopSloped : public ExtrusionLoop`
#[derive(Debug, Clone)]
pub struct ExtrusionLoopSloped {
    pub base: ExtrusionLoop,
    // ExtrusionEntity.hpp:580 `std::vector<ExtrusionPathSloped> starts;`
    pub starts: Vec<ExtrusionPathSloped>,
    // ExtrusionEntity.hpp:581 `std::vector<ExtrusionPathSloped> ends;`
    pub ends: Vec<ExtrusionPathSloped>,
    // ExtrusionEntity.hpp:582 `double target_speed{0.0};`
    pub target_speed: f64,
}

impl ExtrusionLoopSloped {
    // ExtrusionEntity.cpp:514-597 `ExtrusionLoopSloped::ExtrusionLoopSloped(...)`
    pub fn new(
        original_paths: &mut Vec<ExtrusionPath>,
        seam_gap: f64,
        slope_min_length: f64,
        slope_max_segment_length: f64,
        start_slope_ratio: f64,
        role: ExtrusionLoopRole,
    ) -> Self {
        // ExtrusionEntity.cpp:519 — : ExtrusionLoop(role)
        let mut this = ExtrusionLoopSloped {
            base: ExtrusionLoop::new(Vec::new(), role),
            starts: Vec::new(),
            ends: Vec::new(),
            target_speed: 0.0,
        };

        // create slopes
        // ExtrusionEntity.cpp:522-565 — const auto add_slop = [...](const ExtrusionPath &path, const Polyline &poly, double ratio_begin, double ratio_end)
        // Implemented as a closure capturing starts/ends/base via &mut this.
        let add_slop = |this: &mut ExtrusionLoopSloped,
                        path: &ExtrusionPath,
                        poly: &Polyline,
                        ratio_begin: f64,
                        mut ratio_end: f64| {
            // ExtrusionEntity.cpp:523
            if poly.empty() {
                return;
            }

            // Ensure `slope_max_segment_length`
            // ExtrusionEntity.cpp:526-543
            let mut detailed_poly = Polyline::new();
            {
                // ExtrusionEntity.cpp:528
                detailed_poly.append_point(poly.first_point());

                // Recursively split the line into half until no longer than `slope_max_segment_length`
                // ExtrusionEntity.cpp:531-540
                fn handle_line(slope_max_segment_length: f64, detailed_poly: &mut Polyline, line: &Line) {
                    // ExtrusionEntity.cpp:532
                    if line.length() <= slope_max_segment_length {
                        // ExtrusionEntity.cpp:533
                        detailed_poly.append_point(line.b);
                    } else {
                        // Then process left half
                        // ExtrusionEntity.cpp:536
                        handle_line(
                            slope_max_segment_length,
                            detailed_poly,
                            &Line {
                                a: line.a,
                                b: line.midpoint(),
                            },
                        );
                        // Then process right half
                        // ExtrusionEntity.cpp:538
                        handle_line(
                            slope_max_segment_length,
                            detailed_poly,
                            &Line {
                                a: line.midpoint(),
                                b: line.b,
                            },
                        );
                    }
                }

                // ExtrusionEntity.cpp:542
                for l in poly.lines() {
                    handle_line(slope_max_segment_length, &mut detailed_poly, &l);
                }
            }

            // ExtrusionEntity.cpp:545
            this.starts.push(ExtrusionPathSloped::new(
                detailed_poly.clone(),
                path,
                Slope {
                    z_ratio: ratio_begin,
                    e_ratio: ratio_begin,
                    speed_record: 0.0,
                },
                Slope {
                    z_ratio: ratio_end,
                    e_ratio: ratio_end,
                    speed_record: 0.0,
                },
            ));

            // ExtrusionEntity.cpp:547
            if is_approx(ratio_end, 1.0) && seam_gap > 0.0 {
                // Remove the segments that has no extrusion
                // ExtrusionEntity.cpp:549
                let seg_length = detailed_poly.length();
                // ExtrusionEntity.cpp:550
                if seg_length > seam_gap {
                    // Split the segment and remove the last `seam_gap` bit
                    // ExtrusionEntity.cpp:552-554
                    let orig = detailed_poly.clone();
                    let mut tmp = Polyline::new();
                    detailed_poly = Polyline::new();
                    orig.split_at_length(seg_length - seam_gap, &mut detailed_poly, &mut tmp);

                    // ExtrusionEntity.cpp:556
                    ratio_end = lerp(ratio_begin, ratio_end, (seg_length - seam_gap) / seg_length);
                    // ExtrusionEntity.cpp:557
                    debug_assert!(1.0 - ratio_end > EPSILON);
                } else {
                    // Remove the entire segment
                    // ExtrusionEntity.cpp:560
                    detailed_poly.clear();
                }
            }
            // ExtrusionEntity.cpp:563
            if !detailed_poly.empty() {
                this.ends.push(ExtrusionPathSloped::new(
                    detailed_poly,
                    path,
                    Slope {
                        z_ratio: 1.0,
                        e_ratio: 1.0 - ratio_begin,
                        speed_record: 0.0,
                    },
                    Slope {
                        z_ratio: 1.0,
                        e_ratio: 1.0 - ratio_end,
                        speed_record: 0.0,
                    },
                ));
            }
        };

        // ExtrusionEntity.cpp:567
        let mut remaining_length = slope_min_length;

        // ExtrusionEntity.cpp:569
        let mut path_iter: usize = 0;
        // ExtrusionEntity.cpp:570
        let mut start_ratio = start_slope_ratio;
        // ExtrusionEntity.cpp:571
        while path_iter != original_paths.len() && remaining_length > 0.0 {
            // ExtrusionEntity.cpp:572
            let path_len = original_paths[path_iter].length() / SCALING_FACTOR;
            // ExtrusionEntity.cpp:573
            if path_len > remaining_length {
                // Split current path into slope and non-slope part
                // ExtrusionEntity.cpp:575-577
                let mut slope_path = Polyline::new();
                let mut flat_path = Polyline::new();
                original_paths[path_iter].polyline.split_at_length(
                    scale(remaining_length) as f64,
                    &mut slope_path,
                    &mut flat_path,
                );

                // ExtrusionEntity.cpp:579
                let path_copy = original_paths[path_iter].clone();
                add_slop(&mut this, &path_copy, &slope_path, start_ratio, 1.0);
                // ExtrusionEntity.cpp:580
                start_ratio = 1.0;

                // ExtrusionEntity.cpp:582 — paths.emplace_back(std::move(flat_path), *path);
                this.base
                    .paths
                    .push(ExtrusionPath::from_polyline_and(flat_path, &path_copy));
                // ExtrusionEntity.cpp:583
                remaining_length = 0.0;
            } else {
                // BBS: protection for accuracy issues
                // ExtrusionEntity.cpp:586
                remaining_length = if remaining_length - path_len < EPSILON {
                    0.0
                } else {
                    remaining_length - path_len
                };
                // ExtrusionEntity.cpp:587
                let end_ratio = lerp(1.0, start_slope_ratio, remaining_length / slope_min_length);
                // ExtrusionEntity.cpp:588
                let path_copy = original_paths[path_iter].clone();
                let poly_copy = path_copy.polyline.clone();
                add_slop(&mut this, &path_copy, &poly_copy, start_ratio, end_ratio);
                // ExtrusionEntity.cpp:589
                start_ratio = end_ratio;
            }
            path_iter += 1;
        }
        // ExtrusionEntity.cpp:592
        debug_assert!(remaining_length <= 0.0);
        // ExtrusionEntity.cpp:593
        debug_assert!(start_ratio == 1.0);

        // Put remaining flat paths
        // ExtrusionEntity.cpp:596 — paths.insert(paths.end(), path, original_paths.end());
        this.base
            .paths
            .extend_from_slice(&original_paths[path_iter..]);

        this
    }

    // ExtrusionEntity.cpp:599-611 `std::vector<const ExtrusionPath *> ExtrusionLoopSloped::get_all_paths() const`
    pub fn get_all_paths(&self) -> Vec<&ExtrusionPath> {
        // ExtrusionEntity.cpp:601
        let mut r: Vec<&ExtrusionPath> = Vec::new();
        // ExtrusionEntity.cpp:602
        r.reserve(self.starts.len() + self.base.paths.len() + self.ends.len());
        // ExtrusionEntity.cpp:603-604
        for p in &self.starts {
            r.push(&p.path);
        }
        // ExtrusionEntity.cpp:605-606
        for p in &self.base.paths {
            r.push(p);
        }
        // ExtrusionEntity.cpp:607-608
        for p in &self.ends {
            r.push(&p.path);
        }
        // ExtrusionEntity.cpp:610
        r
    }

    // ExtrusionEntity.cpp:325-331 `void ExtrusionLoopSloped::clip_slope(double distance, bool inter_perimeter)`
    // BBS: clipe slope a bit
    pub fn clip_slope(&mut self, distance: f64, _inter_perimeter: bool) {
        // ExtrusionEntity.cpp:329
        self.clip_end(distance);
        // ExtrusionEntity.cpp:330
        self.clip_front(distance * 2.0);
    }

    // ExtrusionEntity.cpp:333-349 `void ExtrusionLoopSloped::clip_end(const double distance)`
    // BBS
    pub fn clip_end(&mut self, distance: f64) {
        // ExtrusionEntity.cpp:336
        let mut clip_dist = distance;
        // ExtrusionEntity.cpp:337 — std::vector<ExtrusionPathSloped> &ends_slope = this->ends;
        // ExtrusionEntity.cpp:338
        while clip_dist > 0.0 && !self.ends.is_empty() {
            // ExtrusionEntity.cpp:339-340
            let len = self.ends.last().unwrap().length();
            // ExtrusionEntity.cpp:341-347
            if len <= clip_dist {
                self.ends.pop();
                clip_dist -= len;
            } else {
                self.ends.last_mut().unwrap().path.polyline.clip_end(clip_dist);
                break;
            }
        }
    }

    // ExtrusionEntity.cpp:351-374 `void ExtrusionLoopSloped::clip_front(const double distance)`
    // BBS
    pub fn clip_front(&mut self, distance: f64) {
        // ExtrusionEntity.cpp:354
        let mut clip_dist = distance;
        // ExtrusionEntity.cpp:355-356
        if self.role() == ExtrusionRole::Perimeter {
            clip_dist = scale(self.slope_path_length()) as f64 * SLOPE_INNER_OUTER_WALL_GAP;
        }

        // ExtrusionEntity.cpp:358 — std::vector<ExtrusionPathSloped> &start_slope = this->starts;

        // ExtrusionEntity.cpp:360 — Polyline front_inward; (unused)
        // ExtrusionEntity.cpp:361
        while distance > 0.0 && !self.starts.is_empty() {
            // ExtrusionEntity.cpp:362-363
            let len = self.starts.first().unwrap().length();
            // ExtrusionEntity.cpp:364-372
            if len <= clip_dist {
                self.starts.remove(0);
                clip_dist -= len;
            } else {
                let first = self.starts.first_mut().unwrap();
                first.path.polyline.reverse();
                first.path.polyline.clip_end(clip_dist);
                first.path.polyline.reverse();
                break;
            }
        }
    }

    // ExtrusionEntity.cpp:376-382 `double ExtrusionLoopSloped::slope_path_length()`
    pub fn slope_path_length(&self) -> f64 {
        // ExtrusionEntity.cpp:377
        let mut total_length = 0.0;
        // ExtrusionEntity.cpp:378-380
        for start_ep in &self.starts {
            total_length += start_ep.length() / SCALING_FACTOR;
        }
        // ExtrusionEntity.cpp:381
        total_length
    }

    // ExtrusionEntity.cpp:384-406 `void ExtrusionLoopSloped::slowdown_slope_speed()`
    // BBS: slowdown slope path seep to get better seam
    pub fn slowdown_slope_speed(&mut self) {
        // ExtrusionEntity.cpp:386
        let speed_base = SLOPE_PATH_RATIO * self.target_speed;
        // ExtrusionEntity.cpp:387
        let mut speed_update = speed_base;
        // ExtrusionEntity.cpp:388
        let mut count_length = 0.0;
        // ExtrusionEntity.cpp:389
        let total_length = self.slope_path_length();

        // ExtrusionEntity.cpp:391-397
        for start_ep in &mut self.starts {
            start_ep.slope_begin.speed_record = speed_update;
            count_length += start_ep.length() / SCALING_FACTOR;
            // mapping speed for each path
            start_ep.slope_end.speed_record =
                speed_base + (self.target_speed - speed_base) * (count_length / total_length);
            speed_update = start_ep.slope_end.speed_record;
        }

        // ExtrusionEntity.cpp:399-405
        for ep_index in 0..self.ends.len() {
            let start_begin = self.starts[self.starts.len() - 1 - ep_index].slope_begin.speed_record;
            let start_end = self.starts[self.starts.len() - 1 - ep_index].slope_end.speed_record;
            let end_ep = &mut self.ends[ep_index];
            end_ep.slope_begin.speed_record = start_end;
            end_ep.slope_end.speed_record = start_begin;
        }
    }

    // ExtrusionEntity.hpp:535 (inherited) `role()`
    pub fn role(&self) -> ExtrusionRole {
        self.base.role()
    }
}

// ===========================================================================
// ExtrusionEntity.cpp:613-689 — role_to_string / string_to_role
// ===========================================================================

// ExtrusionEntity.cpp:613-641 `std::string ExtrusionEntity::role_to_string(ExtrusionRole role)`
pub fn role_to_string(role: ExtrusionRole) -> &'static str {
    // ExtrusionEntity.cpp:615-639
    match role {
        ExtrusionRole::None => "Undefined",
        ExtrusionRole::Perimeter => "Inner wall",
        ExtrusionRole::ExternalPerimeter => "Outer wall",
        ExtrusionRole::OverhangPerimeter => "Overhang wall",
        ExtrusionRole::InternalInfill => "Sparse infill",
        ExtrusionRole::FloatingVerticalShell => "Floating vertical shell",
        ExtrusionRole::SolidInfill => "Internal solid infill",
        ExtrusionRole::TopSolidInfill => "Top surface",
        ExtrusionRole::BottomSurface => "Bottom surface",
        ExtrusionRole::Ironing => "Ironing",
        ExtrusionRole::SupportIroning => "Support ironing",
        ExtrusionRole::BridgeInfill => "Bridge",
        ExtrusionRole::GapFill => "Gap infill",
        ExtrusionRole::Skirt => "Skirt",
        ExtrusionRole::Brim => "Brim",
        ExtrusionRole::SupportMaterial => "Support",
        ExtrusionRole::SupportMaterialInterface => "Support interface",
        ExtrusionRole::SupportTransition => "Support transition",
        ExtrusionRole::WipeTower => "Prime tower",
        ExtrusionRole::Custom => "Custom",
        ExtrusionRole::Mixed => "Multiple",
        ExtrusionRole::Flush => "Flush",
    }
}

// ExtrusionEntity.cpp:643-689 `ExtrusionRole ExtrusionEntity::string_to_role(const std::string_view role)`
pub fn string_to_role(role: &str) -> ExtrusionRole {
    // ExtrusionEntity.cpp:645-688
    if role == "Inner wall" {
        ExtrusionRole::Perimeter
    } else if role == "Outer wall" {
        ExtrusionRole::ExternalPerimeter
    } else if role == "Overhang wall" {
        ExtrusionRole::OverhangPerimeter
    } else if role == "Sparse infill" {
        ExtrusionRole::InternalInfill
    } else if role == "Floating vertical shell" {
        ExtrusionRole::FloatingVerticalShell
    } else if role == "Internal solid infill" {
        ExtrusionRole::SolidInfill
    } else if role == "Top surface" {
        ExtrusionRole::TopSolidInfill
    } else if role == "Bottom surface" {
        ExtrusionRole::BottomSurface
    } else if role == "Ironing" {
        ExtrusionRole::Ironing
    } else if role == "Support ironing" {
        ExtrusionRole::SupportIroning
    } else if role == "Bridge" {
        ExtrusionRole::BridgeInfill
    } else if role == "Gap infill" {
        ExtrusionRole::GapFill
    } else if role == "Skirt" {
        ExtrusionRole::Skirt
    } else if role == "Brim" {
        ExtrusionRole::Brim
    } else if role == "Support" {
        ExtrusionRole::SupportMaterial
    } else if role == "Support interface" {
        ExtrusionRole::SupportMaterialInterface
    } else if role == "Support transition" {
        ExtrusionRole::SupportTransition
    } else if role == "Prime tower" {
        ExtrusionRole::WipeTower
    } else if role == "Custom" {
        ExtrusionRole::Custom
    } else if role == "Multiple" {
        ExtrusionRole::Mixed
    } else if role == "Flush" {
        ExtrusionRole::Flush
    } else {
        ExtrusionRole::None
    }
}

// ===========================================================================
// Free helpers ported from libslic3r.h / Point.hpp used by this file.
// ===========================================================================

/// libslic3r.h:280-285 `template <typename T, typename Number> constexpr inline T lerp(const T& a, const T& b, Number t)`
#[inline]
fn lerp(a: f64, b: f64, t: f64) -> f64 {
    // assert((t >= -EPSILON) && (t <= 1 + EPSILON));
    (1.0 - t) * a + t * b
}

/// libslic3r.h:287-291 `template <typename Number> constexpr inline bool is_approx(Number value, Number test_value, Number precision = EPSILON)`
#[inline]
fn is_approx(value: f64, test_value: f64) -> bool {
    (value - test_value).abs() < EPSILON
}

/// Point.hpp:110-117 `inline double angle(const v1, const v2)` -> atan2(cross2(v1d, v2d), v1d.dot(v2d))
#[inline]
fn angle(v1: Point, v2: Point) -> f64 {
    let v1x = v1.x as f64;
    let v1y = v1.y as f64;
    let v2x = v2.x as f64;
    let v2y = v2.y as f64;
    // cross2(v1, v2) = v1.x*v2.y - v1.y*v2.x ; dot = v1.x*v2.x + v1.y*v2.y
    (v1x * v2y - v1y * v2x).atan2(v1x * v2x + v1y * v2y)
}

/// (unscale(a) - unscale(b)).norm(): C++ unscale<double>(Point) divides each coord by
/// SCALING_FACTOR; the Euclidean norm of the difference in unscaled (mm) units.
#[inline]
fn unscale_point_norm(a: &Point, b: &Point) -> f64 {
    let dx = unscale(a.x - b.x);
    let dy = unscale(a.y - b.y);
    (dx * dx + dy * dy).sqrt()
}

// ===========================================================================
// ExtrusionEntity trait (Rust-side virtual dispatch surface)
// ===========================================================================

/// Base trait for all extrusion entities (mirrors `class ExtrusionEntity`).
/// ExtrusionEntity.hpp:138-208
pub trait ExtrusionEntity {
    // ExtrusionEntity.hpp:163 `virtual ExtrusionRole role() const = 0;`
    fn role(&self) -> ExtrusionRole;
    // ExtrusionEntity.hpp:164 `virtual bool is_collection() const { return false; }`
    fn is_collection(&self) -> bool {
        false
    }
    // ExtrusionEntity.hpp:165 `virtual bool is_loop() const { return false; }`
    fn is_loop(&self) -> bool {
        false
    }
    // ExtrusionEntity.hpp:166 `virtual bool can_reverse() const { return true; }`
    fn can_reverse(&self) -> bool {
        true
    }
    // ExtrusionEntity.hpp:173 `virtual void reverse() = 0;`
    fn reverse(&mut self);
    // ExtrusionEntity.hpp:174 `virtual const Point& first_point() const = 0;`
    fn first_point(&self) -> Point;
    // ExtrusionEntity.hpp:175 `virtual const Point& last_point() const = 0;`
    fn last_point(&self) -> Point;
    // ExtrusionEntity.hpp:189 `virtual Polyline as_polyline() const = 0;`
    fn as_polyline(&self) -> Polyline;
    // ExtrusionEntity.hpp:190 `virtual void collect_polylines(Polylines &dst) const = 0;`
    fn collect_polylines(&self, dst: &mut Polylines);
    // ExtrusionEntity.hpp:193 `virtual double length() const = 0;`
    fn length(&self) -> CoordF;
    // ExtrusionEntity.hpp:194 `virtual double total_volume() const = 0;`
    fn total_volume(&self) -> CoordF;
    // ExtrusionEntity.hpp:188 `virtual double min_mm3_per_mm() const = 0;`
    fn min_mm3_per_mm(&self) -> CoordF;
}

impl ExtrusionEntity for ExtrusionPath {
    fn role(&self) -> ExtrusionRole {
        self.role
    }
    fn can_reverse(&self) -> bool {
        ExtrusionPath::can_reverse(self)
    }
    fn reverse(&mut self) {
        ExtrusionPath::reverse(self);
    }
    fn first_point(&self) -> Point {
        ExtrusionPath::first_point(self)
    }
    fn last_point(&self) -> Point {
        ExtrusionPath::last_point(self)
    }
    fn as_polyline(&self) -> Polyline {
        ExtrusionPath::as_polyline(self)
    }
    // ExtrusionEntity.hpp:346 `if (! this->polyline.empty()) dst.emplace_back(this->polyline);`
    fn collect_polylines(&self, dst: &mut Polylines) {
        if !self.polyline.empty() {
            dst.push(self.polyline.clone());
        }
    }
    fn length(&self) -> CoordF {
        ExtrusionPath::length(self)
    }
    fn total_volume(&self) -> CoordF {
        ExtrusionPath::total_volume(self)
    }
    fn min_mm3_per_mm(&self) -> CoordF {
        ExtrusionPath::min_mm3_per_mm(self)
    }
}

impl ExtrusionEntity for ExtrusionLoop {
    fn role(&self) -> ExtrusionRole {
        ExtrusionLoop::role(self)
    }
    // ExtrusionEntity.hpp:506 `bool is_loop() const override{ return true; }`
    fn is_loop(&self) -> bool {
        true
    }
    // ExtrusionEntity.hpp:507 `bool can_reverse() const override { return false; }`
    fn can_reverse(&self) -> bool {
        false
    }
    fn reverse(&mut self) {
        ExtrusionLoop::reverse(self);
    }
    fn first_point(&self) -> Point {
        ExtrusionLoop::first_point(self)
    }
    fn last_point(&self) -> Point {
        ExtrusionLoop::last_point(self)
    }
    fn as_polyline(&self) -> Polyline {
        ExtrusionLoop::as_polyline(self)
    }
    // ExtrusionEntity.hpp:552 `Polyline pl = this->as_polyline(); if (! pl.empty()) dst.emplace_back(std::move(pl));`
    fn collect_polylines(&self, dst: &mut Polylines) {
        let pl = self.as_polyline();
        if !pl.empty() {
            dst.push(pl);
        }
    }
    fn length(&self) -> CoordF {
        ExtrusionLoop::length(self)
    }
    fn total_volume(&self) -> CoordF {
        ExtrusionLoop::total_volume(self)
    }
    fn min_mm3_per_mm(&self) -> CoordF {
        ExtrusionLoop::min_mm3_per_mm(self)
    }
}

impl ExtrusionEntity for ExtrusionMultiPath {
    fn role(&self) -> ExtrusionRole {
        ExtrusionMultiPath::role(self)
    }
    fn can_reverse(&self) -> bool {
        ExtrusionMultiPath::can_reverse(self)
    }
    fn reverse(&mut self) {
        ExtrusionMultiPath::reverse(self);
    }
    fn first_point(&self) -> Point {
        ExtrusionMultiPath::first_point(self)
    }
    fn last_point(&self) -> Point {
        ExtrusionMultiPath::last_point(self)
    }
    fn as_polyline(&self) -> Polyline {
        ExtrusionMultiPath::as_polyline(self)
    }
    // ExtrusionEntity.hpp:479 `Polyline pl = this->as_polyline(); if (! pl.empty()) dst.emplace_back(std::move(pl));`
    fn collect_polylines(&self, dst: &mut Polylines) {
        let pl = self.as_polyline();
        if !pl.empty() {
            dst.push(pl);
        }
    }
    fn length(&self) -> CoordF {
        ExtrusionMultiPath::length(self)
    }
    fn total_volume(&self) -> CoordF {
        ExtrusionMultiPath::total_volume(self)
    }
    fn min_mm3_per_mm(&self) -> CoordF {
        ExtrusionMultiPath::min_mm3_per_mm(self)
    }
}

// ===========================================================================
// ExtrusionEntityCollection (struct lives here; methods in
// crate::extrusion_entity_collection mirror ExtrusionEntityCollection.cpp).
// ===========================================================================

/// A collection of extrusion entities (paths, loops, or nested collections).
/// ExtrusionEntityCollection.hpp `class ExtrusionEntityCollection : public ExtrusionEntity`
#[derive(Debug, Clone)]
pub struct ExtrusionEntityCollection {
    pub entities: Vec<ExtrusionEntityType>,
    pub no_sort: bool,
    pub orig_indices: Vec<usize>,
}

/// Enum to hold different types of extrusion entities (replaces `ExtrusionEntity*`).
#[derive(Debug, Clone)]
pub enum ExtrusionEntityType {
    Path(ExtrusionPath),
    Loop(ExtrusionLoop),
    Collection(Box<ExtrusionEntityCollection>),
}

impl ExtrusionEntityCollection {
    pub fn new() -> Self {
        Self {
            entities: Vec::new(),
            no_sort: false,
            orig_indices: Vec::new(),
        }
    }

    pub fn append(&mut self, entity: ExtrusionEntityType) {
        self.entities.push(entity);
    }

    pub fn append_path(&mut self, path: ExtrusionPath) {
        self.entities.push(ExtrusionEntityType::Path(path));
    }

    pub fn append_loop(&mut self, loop_: ExtrusionLoop) {
        self.entities.push(ExtrusionEntityType::Loop(loop_));
    }

    pub fn append_collection(&mut self, collection: ExtrusionEntityCollection) {
        self.entities
            .push(ExtrusionEntityType::Collection(Box::new(collection)));
    }

    pub fn is_empty(&self) -> bool {
        self.entities.is_empty()
    }

    pub fn len(&self) -> usize {
        self.entities.len()
    }

    pub fn clear(&mut self) {
        self.entities.clear();
        self.orig_indices.clear();
    }

    // NOTE: `ExtrusionEntityCollection::role()` (which collapses to erMixed when child
    // roles differ) is ported in `crate::extrusion_entity_collection` alongside the rest
    // of ExtrusionEntityCollection.cpp; not redefined here to avoid a duplicate method.

    // ExtrusionEntityCollection.cpp:67-75 `void ExtrusionEntityCollection::reverse()`
    pub fn reverse(&mut self) {
        // for (ExtrusionEntity *ptr : this->entities)
        //     // Don't reverse it if it's a loop, as it doesn't change anything in terms of elements ordering
        //     // and caller might rely on winding order
        //     if (! ptr->is_loop())
        //         ptr->reverse();
        for entity in &mut self.entities {
            match entity {
                ExtrusionEntityType::Path(path) => path.reverse(),
                ExtrusionEntityType::Loop(_) => {}
                ExtrusionEntityType::Collection(coll) => coll.reverse(),
            }
        }
        // std::reverse(this->entities.begin(), this->entities.end());
        self.entities.reverse();
    }

    pub fn first_point(&self) -> Option<Point> {
        if self.entities.is_empty() {
            return None;
        }
        Some(match &self.entities[0] {
            ExtrusionEntityType::Path(path) => path.first_point(),
            ExtrusionEntityType::Loop(loop_) => loop_.first_point(),
            ExtrusionEntityType::Collection(coll) => coll.first_point()?,
        })
    }

    pub fn last_point(&self) -> Option<Point> {
        if self.entities.is_empty() {
            return None;
        }
        Some(match self.entities.last().unwrap() {
            ExtrusionEntityType::Path(path) => path.last_point(),
            ExtrusionEntityType::Loop(loop_) => loop_.last_point(),
            ExtrusionEntityType::Collection(coll) => coll.last_point()?,
        })
    }

    /// Rust convenience helper (NOT a direct C++ port): returns references to all
    /// `ExtrusionPath`s contained anywhere in this collection, descending into loops
    /// and nested collections. For the faithful C++ `ExtrusionEntityCollection::flatten`
    /// (which returns an `ExtrusionEntityCollection` and keeps loops intact), see
    /// `crate::extrusion_entity_collection`.
    pub fn flatten_paths(&self) -> Vec<&ExtrusionPath> {
        let mut result = Vec::new();
        for entity in &self.entities {
            match entity {
                ExtrusionEntityType::Path(path) => result.push(path),
                ExtrusionEntityType::Loop(loop_) => {
                    for path in &loop_.paths {
                        result.push(path);
                    }
                }
                ExtrusionEntityType::Collection(coll) => {
                    result.extend(coll.flatten_paths());
                }
            }
        }
        result
    }

    pub fn collect_polylines(&self) -> Polylines {
        let mut result = Polylines::new();
        for entity in &self.entities {
            match entity {
                ExtrusionEntityType::Path(path) => result.push(path.polyline.clone()),
                ExtrusionEntityType::Loop(loop_) => {
                    for path in &loop_.paths {
                        result.push(path.polyline.clone());
                    }
                }
                ExtrusionEntityType::Collection(coll) => {
                    result.extend(coll.collect_polylines());
                }
            }
        }
        result
    }

    pub fn length(&self) -> CoordF {
        self.entities
            .iter()
            .map(|e| match e {
                ExtrusionEntityType::Path(path) => path.length(),
                ExtrusionEntityType::Loop(loop_) => loop_.length(),
                ExtrusionEntityType::Collection(coll) => coll.length(),
            })
            .sum()
    }

    pub fn total_volume(&self) -> CoordF {
        self.entities
            .iter()
            .map(|e| match e {
                ExtrusionEntityType::Path(path) => path.total_volume(),
                ExtrusionEntityType::Loop(loop_) => loop_.total_volume(),
                ExtrusionEntityType::Collection(coll) => coll.total_volume(),
            })
            .sum()
    }
}

impl Default for ExtrusionEntityCollection {
    fn default() -> Self {
        Self::new()
    }
}

// Implement IntoIterator to allow for-in loops on references
impl<'a> IntoIterator for &'a ExtrusionEntityCollection {
    type Item = &'a ExtrusionEntityType;
    type IntoIter = std::slice::Iter<'a, ExtrusionEntityType>;

    fn into_iter(self) -> Self::IntoIter {
        self.entities.iter()
    }
}

// Implement IntoIterator for owned collection
impl IntoIterator for ExtrusionEntityCollection {
    type Item = ExtrusionEntityType;
    type IntoIter = std::vec::IntoIter<ExtrusionEntityType>;

    fn into_iter(self) -> Self::IntoIter {
        self.entities.into_iter()
    }
}

// ===========================================================================
// ExtrusionEntity.hpp:595-757 — inline free helpers
// ===========================================================================

/// ExtrusionEntity.hpp:647-656 `inline void extrusion_entities_append_paths(ExtrusionEntitiesPtr &dst, Polylines &polylines, ExtrusionRole role, double mm3_per_mm, float width, float height)`
pub fn extrusion_entities_append_paths(
    dst: &mut Vec<ExtrusionEntityType>,
    polylines: Vec<Polyline>,
    role: ExtrusionRole,
    mm3_per_mm: CoordF,
    width: f32,
    height: f32,
) {
    // ExtrusionEntity.hpp:649
    dst.reserve(dst.len() + polylines.len());
    // ExtrusionEntity.hpp:650-655
    for polyline in polylines {
        if polyline.is_valid() {
            let mut extrusion_path =
                ExtrusionPath::with_params(role, mm3_per_mm, width as CoordF, height as CoordF, false);
            extrusion_path.polyline = polyline;
            dst.push(ExtrusionEntityType::Path(extrusion_path));
        }
    }
}
