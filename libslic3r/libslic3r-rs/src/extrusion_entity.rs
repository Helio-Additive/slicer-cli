//! Extrusion entity types and traits.
//!
//! This module corresponds to:
//! - `src/libslic3r/ExtrusionEntity.hpp`
//! - `src/libslic3r/ExtrusionEntity.cpp`
//!
//! Extrusion entities represent toolpath segments with associated printing parameters.

use crate::geometry::{Point, Polygon, Polyline, Polylines};
use crate::CoordF;

/// Each ExtrusionRole value identifies a distinct set of { extruder, speed }
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
}

impl ExtrusionRole {
    /// Convert role to string matching C++ ExtrusionEntity::role_to_string()
    /// ExtrusionEntity.cpp:613-641
    /// C++ uses localized strings with L() macro
    pub fn to_string(&self) -> &'static str {
        match self {
            ExtrusionRole::None => "Undefined",
            ExtrusionRole::Perimeter => "Inner wall",
            ExtrusionRole::ExternalPerimeter => "Outer wall",
            ExtrusionRole::OverhangPerimeter => "Overhang wall",
            ExtrusionRole::InternalInfill => "Sparse infill",
            ExtrusionRole::SolidInfill => "Internal solid infill",
            ExtrusionRole::FloatingVerticalShell => "Floating vertical shell",
            ExtrusionRole::TopSolidInfill => "Top surface",
            ExtrusionRole::BottomSurface => "Bottom surface",
            ExtrusionRole::Ironing => "Ironing",
            ExtrusionRole::BridgeInfill => "Bridge",
            ExtrusionRole::GapFill => "Gap infill",
            ExtrusionRole::Skirt => "Skirt",
            ExtrusionRole::Brim => "Brim",
            ExtrusionRole::SupportMaterial => "Support",
            ExtrusionRole::SupportMaterialInterface => "Support interface",
            ExtrusionRole::SupportTransition => "Support transition",
            ExtrusionRole::SupportIroning => "Support ironing",
            ExtrusionRole::WipeTower => "Prime tower",
            ExtrusionRole::Custom => "Custom",
            ExtrusionRole::Flush => "Flush",
            ExtrusionRole::Mixed => "Multiple",
        }
    }

    /// Parse role from string matching C++ ExtrusionEntity::string_to_role()
    /// ExtrusionEntity.cpp:643-688
    pub fn from_string(s: &str) -> Option<Self> {
        match s {
            "Undefined" => Some(ExtrusionRole::None),
            "Inner wall" => Some(ExtrusionRole::Perimeter),
            "Outer wall" => Some(ExtrusionRole::ExternalPerimeter),
            "Overhang wall" => Some(ExtrusionRole::OverhangPerimeter),
            "Sparse infill" => Some(ExtrusionRole::InternalInfill),
            "Internal solid infill" => Some(ExtrusionRole::SolidInfill),
            "Floating vertical shell" => Some(ExtrusionRole::FloatingVerticalShell),
            "Top surface" => Some(ExtrusionRole::TopSolidInfill),
            "Bottom surface" => Some(ExtrusionRole::BottomSurface),
            "Ironing" => Some(ExtrusionRole::Ironing),
            "Bridge" => Some(ExtrusionRole::BridgeInfill),
            "Gap infill" => Some(ExtrusionRole::GapFill),
            "Skirt" => Some(ExtrusionRole::Skirt),
            "Brim" => Some(ExtrusionRole::Brim),
            "Support" => Some(ExtrusionRole::SupportMaterial),
            "Support interface" => Some(ExtrusionRole::SupportMaterialInterface),
            "Support transition" => Some(ExtrusionRole::SupportTransition),
            "Support ironing" => Some(ExtrusionRole::SupportIroning),
            "Prime tower" => Some(ExtrusionRole::WipeTower),
            "Custom" => Some(ExtrusionRole::Custom),
            "Flush" => Some(ExtrusionRole::Flush),
            "Multiple" => Some(ExtrusionRole::Mixed),
            _ => None,
        }
    }
}

/// Helper functions for ExtrusionRole classification
impl ExtrusionRole {
    pub fn is_perimeter(&self) -> bool {
        matches!(
            self,
            ExtrusionRole::Perimeter
                | ExtrusionRole::ExternalPerimeter
                | ExtrusionRole::OverhangPerimeter
        )
    }

    pub fn is_infill(&self) -> bool {
        matches!(
            self,
            ExtrusionRole::BridgeInfill
                | ExtrusionRole::InternalInfill
                | ExtrusionRole::SolidInfill
                | ExtrusionRole::FloatingVerticalShell
                | ExtrusionRole::TopSolidInfill
                | ExtrusionRole::BottomSurface
                | ExtrusionRole::Ironing
        )
    }

    pub fn is_top_surface(&self) -> bool {
        matches!(self, ExtrusionRole::TopSolidInfill)
    }

    pub fn is_solid_infill(&self) -> bool {
        matches!(
            self,
            ExtrusionRole::BridgeInfill
                | ExtrusionRole::SolidInfill
                | ExtrusionRole::FloatingVerticalShell
                | ExtrusionRole::TopSolidInfill
                | ExtrusionRole::BottomSurface
                | ExtrusionRole::Ironing
        )
    }

    pub fn is_bridge(&self) -> bool {
        matches!(
            self,
            ExtrusionRole::BridgeInfill | ExtrusionRole::OverhangPerimeter
        )
    }

    pub fn is_support(&self) -> bool {
        matches!(
            self,
            ExtrusionRole::SupportMaterial
                | ExtrusionRole::SupportMaterialInterface
                | ExtrusionRole::SupportTransition
                | ExtrusionRole::SupportIroning
        )
    }
}

/// Special flags describing loop customization
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CustomizeFlag {
    None,
    CircleCompensation, // shaft hole tolerance compensation
    FloatingVerticalShell,
}

/// Special flags describing loop role
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ExtrusionLoopRole(pub u8);

impl ExtrusionLoopRole {
    pub const DEFAULT: Self = Self(1 << 0);
    pub const CONTOUR_INTERNAL_PERIMETER: Self = Self(1 << 1);
    pub const SKIRT: Self = Self(1 << 2);
    pub const PERIMETER_HOLE: Self = Self(1 << 3);
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

/// A single extrusion path with constant width, height, and role
#[derive(Debug, Clone)]
pub struct ExtrusionPath {
    pub polyline: Polyline,
    pub role: ExtrusionRole,
    pub mm3_per_mm: CoordF,
    pub width: CoordF,
    pub height: CoordF,
    pub overhang_degree: i32,
    pub curve_degree: i32,
    pub customize_flag: CustomizeFlag,
}

impl ExtrusionPath {
    pub fn new(role: ExtrusionRole) -> Self {
        Self {
            polyline: Polyline::new(),
            role,
            mm3_per_mm: 0.0,
            width: 0.0,
            height: 0.0,
            overhang_degree: 0,
            curve_degree: 0,
            customize_flag: CustomizeFlag::None,
        }
    }

    pub fn first_point(&self) -> Point {
        self.polyline.first_point()
    }

    pub fn last_point(&self) -> Point {
        self.polyline.last_point()
    }

    pub fn length(&self) -> CoordF {
        self.polyline.length()
    }

    pub fn reverse(&mut self) {
        self.polyline.reverse();
    }

    pub fn set_customize_flag(&mut self, flag: CustomizeFlag) {
        self.customize_flag = flag;
    }

    pub fn get_customize_flag(&self) -> CustomizeFlag {
        self.customize_flag
    }

    /// Calculate the total extrusion volume for this path
    pub fn total_volume(&self) -> CoordF {
        self.mm3_per_mm * self.length()
    }

    /// Set curve degree with clamping to valid range [0, 10]
    /// ExtrusionEntity.hpp:360-362
    /// C++: void set_curve_degree(int curve) {
    /// C++:     curve_degree = (curve < 0)?0:(curve > 10 ? 10 : curve);
    /// C++: }
    pub fn set_curve_degree(&mut self, curve: i32) {
        self.curve_degree = if curve < 0 {
            0
        } else if curve > 10 {
            10
        } else {
            curve
        };
    }
}

/// A closed loop composed of multiple extrusion paths
#[derive(Debug, Clone)]
pub struct ExtrusionLoop {
    pub paths: Vec<ExtrusionPath>,
    pub role: ExtrusionLoopRole,
    pub customize_flag: CustomizeFlag,
}

impl ExtrusionLoop {
    pub fn new(paths: Vec<ExtrusionPath>, role: ExtrusionLoopRole) -> Self {
        Self {
            paths,
            role,
            customize_flag: CustomizeFlag::None,
        }
    }

    pub fn new_with_flag(
        paths: Vec<ExtrusionPath>,
        role: ExtrusionLoopRole,
        flag: CustomizeFlag,
    ) -> Self {
        Self {
            paths,
            role,
            customize_flag: flag,
        }
    }

    pub fn reverse(&mut self) {
        self.paths.reverse();
        for path in &mut self.paths {
            path.reverse();
        }
    }

    pub fn make_clockwise(&mut self) {
        // In a closed loop, check if the polygon formed by the paths is counter-clockwise
        // If so, reverse to make it clockwise
        let polygon = self.as_polygon();
        if polygon.is_counter_clockwise() {
            self.reverse();
        }
    }

    pub fn make_counter_clockwise(&mut self) {
        // Check if clockwise and reverse if so
        let polygon = self.as_polygon();
        if !polygon.is_counter_clockwise() {
            self.reverse();
        }
    }

    pub fn as_polygon(&self) -> Polygon {
        let mut points = Vec::new();
        for path in &self.paths {
            points.extend_from_slice(path.polyline.points());
        }
        Polygon::from_points(points)
    }

    pub fn as_polyline(&self) -> Polyline {
        let mut points = Vec::new();
        for path in &self.paths {
            points.extend_from_slice(path.polyline.points());
        }
        Polyline::from_points(points)
    }

    pub fn first_point(&self) -> Point {
        self.paths[0].first_point()
    }

    pub fn last_point(&self) -> Point {
        self.paths.last().unwrap().last_point()
    }

    pub fn length(&self) -> CoordF {
        self.paths.iter().map(|p| p.length()).sum()
    }

    pub fn total_volume(&self) -> CoordF {
        self.paths.iter().map(|p| p.total_volume()).sum()
    }

    pub fn set_customize_flag(&mut self, flag: CustomizeFlag) {
        self.customize_flag = flag;
        for path in &mut self.paths {
            path.set_customize_flag(flag);
        }
    }
}

/// Base trait for all extrusion entities
pub trait ExtrusionEntity {
    fn role(&self) -> ExtrusionRole;
    fn is_collection(&self) -> bool {
        false
    }
    fn is_loop(&self) -> bool {
        false
    }
    fn can_reverse(&self) -> bool {
        true
    }
    fn reverse(&mut self);
    fn first_point(&self) -> Point;
    fn last_point(&self) -> Point;
    fn as_polyline(&self) -> Polyline;
    fn collect_polylines(&self, dst: &mut Polylines);
    fn length(&self) -> CoordF;
    fn total_volume(&self) -> CoordF;
    fn min_mm3_per_mm(&self) -> CoordF;
}

impl ExtrusionEntity for ExtrusionPath {
    fn role(&self) -> ExtrusionRole {
        self.role
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
        self.polyline.clone()
    }

    fn collect_polylines(&self, dst: &mut Polylines) {
        dst.push(self.polyline.clone());
    }

    fn length(&self) -> CoordF {
        ExtrusionPath::length(self)
    }

    fn total_volume(&self) -> CoordF {
        ExtrusionPath::total_volume(self)
    }

    fn min_mm3_per_mm(&self) -> CoordF {
        self.mm3_per_mm
    }
}

impl ExtrusionEntity for ExtrusionLoop {
    fn role(&self) -> ExtrusionRole {
        // If all paths have the same role, return that
        // Otherwise return Mixed
        if self.paths.is_empty() {
            return ExtrusionRole::None;
        }
        let first_role = self.paths[0].role;
        if self.paths.iter().all(|p| p.role == first_role) {
            first_role
        } else {
            ExtrusionRole::Mixed
        }
    }

    fn is_loop(&self) -> bool {
        true
    }

    fn can_reverse(&self) -> bool {
        true
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

    fn collect_polylines(&self, dst: &mut Polylines) {
        for path in &self.paths {
            dst.push(path.polyline.clone());
        }
    }

    fn length(&self) -> CoordF {
        ExtrusionLoop::length(self)
    }

    fn total_volume(&self) -> CoordF {
        ExtrusionLoop::total_volume(self)
    }

    fn min_mm3_per_mm(&self) -> CoordF {
        self.paths
            .iter()
            .map(|p| p.mm3_per_mm)
            .min_by(|a, b| a.partial_cmp(b).unwrap())
            .unwrap_or(0.0)
    }
}

/// A collection of extrusion entities (paths, loops, or nested collections)
#[derive(Debug, Clone)]
pub struct ExtrusionEntityCollection {
    pub entities: Vec<ExtrusionEntityType>,
    pub no_sort: bool,
    pub orig_indices: Vec<usize>,
}

/// Enum to hold different types of extrusion entities
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

    pub fn reverse(&mut self) {
        self.entities.reverse();
        for entity in &mut self.entities {
            match entity {
                ExtrusionEntityType::Path(path) => path.reverse(),
                ExtrusionEntityType::Loop(loop_) => loop_.reverse(),
                ExtrusionEntityType::Collection(coll) => coll.reverse(),
            }
        }
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

    pub fn flatten(&self) -> Vec<&ExtrusionPath> {
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
                    result.extend(coll.flatten());
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

/// Helper to chain and reorder extrusion paths for optimal travel
pub fn chain_and_reorder_extrusion_paths(paths: &mut Vec<ExtrusionPath>, start_near: &Point) {
    if paths.is_empty() {
        return;
    }

    // Simple greedy nearest-neighbor chaining
    let mut ordered = Vec::new();
    let mut used = vec![false; paths.len()];
    let mut current_point = *start_near;

    for _ in 0..paths.len() {
        let mut best_idx = None;
        let mut best_dist = CoordF::INFINITY;
        let mut best_reverse = false;

        for (i, path) in paths.iter().enumerate() {
            if used[i] {
                continue;
            }

            let dist_first = current_point.distance(&path.first_point());
            let dist_last = current_point.distance(&path.last_point());

            if dist_first < best_dist {
                best_dist = dist_first;
                best_idx = Some(i);
                best_reverse = false;
            }
            if dist_last < best_dist {
                best_dist = dist_last;
                best_idx = Some(i);
                best_reverse = true;
            }
        }

        if let Some(idx) = best_idx {
            used[idx] = true;
            let mut path = paths[idx].clone();
            if best_reverse {
                path.reverse();
            }
            current_point = path.last_point();
            ordered.push(path);
        }
    }

    *paths = ordered;
}

/// Append polylines as extrusion paths to an entity collection
/// ExtrusionEntity.hpp:647-656
/// C++: inline void extrusion_entities_append_paths(ExtrusionEntitiesPtr &dst, Polylines &polylines, ExtrusionRole role, double mm3_per_mm, float width, float height)
pub fn extrusion_entities_append_paths(
    dst: &mut Vec<ExtrusionEntityType>,
    polylines: Vec<Polyline>,
    role: ExtrusionRole,
    mm3_per_mm: CoordF,
    width: f32,
    height: f32,
) {
    // ExtrusionEntity.hpp:648
    // C++: dst.reserve(dst.size() + polylines.size());
    dst.reserve(dst.len() + polylines.len());

    // ExtrusionEntity.hpp:649-654
    // C++: for (Polyline &polyline : polylines)
    // C++:     if (polyline.is_valid()) {
    // C++:         ExtrusionPath *extrusion_path = new ExtrusionPath(role, mm3_per_mm, width, height);
    // C++:         dst.push_back(extrusion_path);
    // C++:         extrusion_path->polyline = polyline;
    // C++:     }
    for polyline in polylines {
        if polyline.is_valid() {
            let mut extrusion_path = ExtrusionPath::new(role);
            extrusion_path.mm3_per_mm = mm3_per_mm;
            extrusion_path.width = width as CoordF;
            extrusion_path.height = height as CoordF;
            extrusion_path.polyline = polyline;
            dst.push(ExtrusionEntityType::Path(extrusion_path));
        }
    }
}
